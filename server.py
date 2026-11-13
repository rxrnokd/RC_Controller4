"""One HTTP/WebSocket endpoint on Pi OS; local Unix socket for the ROS container."""
import argparse
import asyncio
import contextlib
import ipaddress
import json
import logging
from pathlib import Path
import time
from urllib.parse import urlsplit

from core import Controller, Odometry, drive_errors
from hardware import Hardware

ROOT = Path(__file__).resolve().parent


def dumps(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':'))


def network_allowed(c, remote, host, origin=None):
    try:
        address = ipaddress.ip_address(remote)
        if not any(address in ipaddress.ip_network(n) for n in c['network']['allowed_subnets']):
            return False
        if host not in c['network']['allowed_hosts']:
            return False
        if origin:
            parsed = urlsplit(origin)
            if parsed.scheme != 'http' or parsed.netloc != host:
                return False
        return True
    except (ValueError, TypeError):
        return False


class Service:
    def __init__(self, config, path):
        self.c = config
        self.control = Controller(config)
        self.hardware = Hardware(config, path)
        self.odom = Odometry(config)
        self.clients = {}
        self.state = {}
        self.map = None
        self.bridge = None
        self.ros = {'tracking': False, 'reason': 'ROS bridge disconnected', 'pose': None}
        self.ros_at = -float('inf')
        self.socket_path = ROOT / config['ros']['socket']
        self.tasks = []
        self.failed = False

    async def start(self, app):
        self.socket_path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        if self.socket_path.exists():
            # Never silently remove another live service's socket.
            try:
                _, writer = await asyncio.open_unix_connection(str(self.socket_path))
            except (ConnectionRefusedError, FileNotFoundError):
                self.socket_path.unlink(missing_ok=True)
            else:
                writer.close()
                raise RuntimeError('ROS socket already in use')
        self.unix = await asyncio.start_unix_server(self.ros_client, str(self.socket_path), limit=16 * 1024 * 1024)
        self.socket_path.chmod(0o600)
        self.tasks = [asyncio.create_task(self.loop()), asyncio.create_task(self.broadcast())]

    async def close(self, app):
        self.control.stop('server shutdown', True)
        self.hardware.close()
        for task in self.tasks:
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        for ws in list(self.clients):
            await ws.close()
        if self.bridge:
            self.bridge.close()
        self.unix.close()
        await self.unix.wait_closed()
        self.socket_path.unlink(missing_ok=True)

    async def loop(self):
        last = time.monotonic()
        try:
            while True:
                now = time.monotonic()
                dt, last = now - last, now
                sensors = self.hardware.sample()
                self.control.feedback = sensors['motors']
                # RPM freshness and position-counter availability are different.
                # After real edges were observed, unchanged counters at commanded rest
                # can maintain odom; never do so while motion is requested or applied.
                resting = self.control.target['value'] == 0 and all(x == 0 for x in self.control.output)
                for m in sensors['motors']:
                    m['odom_valid'] = bool(m.get('valid') or (resting and m.get('edge_age_s') is not None
                        and m.get('turns') is not None and m.get('invalid_transitions') == 0))
                    m['position_quality'] = 'fresh_edges' if m.get('valid') else ('unchanged_at_commanded_rest' if m['odom_valid'] else 'unavailable')
                odom = self.odom.update(sensors['motors'], now)
                ros = dict(self.ros)
                if now - self.ros_at > self.c['ros']['freshness_s']:
                    ros.update(tracking=False, pose=None, reason='ROS bridge stale/disconnected')
                if odom is None:
                    ros.update(tracking=False, pose=None, reason='odometry unavailable / continuity lost')
                threshold = self.c['auto_park']['obstacle_stop_cm']
                clear = threshold is not None and all(u['cm'] is not None and u['cm'] > threshold and u['status'] == 'ok' for u in sensors['ultrasonic'])
                self.control.auto_ready = bool(ros.get('tracking') and odom and clear and all(m['valid'] for m in sensors['motors']))
                if dt > self.c['safety']['command_timeout_s']:
                    self.control.stop('control loop deadline missed', True)
                self.control.tick(min(dt, 0.05))
                self.hardware.apply(self.control)
                for i, motor in enumerate(sensors['motors']):
                    motor.update(pwm=self.control.output[i], target_pwm=self.control.requested[i])
                self.state = {'v': 1, 'type': 'state', 'server_time': now, 'mode': self.control.mode,
                              'estop': self.control.estop, 'armed': self.control.armed, 'reason': self.control.reason,
                              'control_taken': self.control.owner is not None, 'source': self.control.role,
                              'target': self.control.target, 'pwm_limit': self.control.limit,
                              'configuration_errors': drive_errors(self.c), 'sensors': sensors,
                              'servo': {'command_rad': self.control.angle, 'target_rad': self.control.target['steering_rad'],
                                        'measured_rad': None, 'pulse_us': self.control.pulse},
                              'odom': odom, 'odom_continuity_lost': self.odom.fault, 'ros': ros,
                              'auto_ready': self.control.can_auto(),
                              'limits': {'pwm': self.c['drive']['pwm_limit'] or 0,
                                         'left_rad': self.c['servo']['left_rad'] or 0,
                                         'right_rad': self.c['servo']['right_rad'] or 0,
                                         'speed_m_s': self.c['drive']['max_speed_m_s'] or 0}}
                await asyncio.sleep(0.02)
        except asyncio.CancelledError:
            raise
        except Exception:
            logging.exception('Control loop failed; outputs disabled')
            self.control.stop('hardware/control fault; restart required', True)
            self.failed = True
            self.state.update(armed=False, estop=True, reason=self.control.reason)
            self.hardware.close()
            # No restart or automatic re-arm on hardware failure.
            for ws in list(self.clients):
                await ws.close(code=1011, message=b'hardware fault')

    async def broadcast(self):
        while True:
            for ws, data in list(self.clients.items()):
                if data['queue'].full():
                    self.control.disconnect(ws)
                    asyncio.create_task(ws.close(code=1013, message=b'slow client'))
                    continue
                payload = dict(self.state, you_control=ws == self.control.owner)
                data['queue'].put_nowait(payload)
                if self.map and data['map_revision'] != self.map['revision']:
                    if not data['queue'].full():
                        data['queue'].put_nowait(self.map)
                        data['map_revision'] = self.map['revision']
            await asyncio.sleep(0.1)

    async def websocket(self, request):
        from aiohttp import web, WSMsgType
        ws = web.WebSocketResponse(heartbeat=1, max_msg_size=4096)
        await ws.prepare(request)
        data = {'queue': asyncio.Queue(maxsize=4), 'map_revision': None}
        self.clients[ws] = data
        async def sender():
            while True:
                item = await data['queue'].get()
                await asyncio.wait_for(ws.send_str(dumps(item)), timeout=0.3)
        async def safe_sender():
            try:
                await sender()
            except (Exception, asyncio.CancelledError):
                self.control.disconnect(ws)
                await ws.close()
        task = asyncio.create_task(safe_sender())
        try:
            async for event in ws:
                if event.type != WSMsgType.TEXT:
                    break
                try:
                    message = None
                    message = json.loads(event.data)
                    if self.failed:
                        raise ValueError('hardware/control fault: restart required')
                    self.control.message(ws, message)
                    reply = {'v': 1, 'type': 'ack', 'request': message.get('type')}
                except (ValueError, TypeError, KeyError) as exc:
                    # Malformed commands from the owner immediately stop existing motion.
                    if ws == self.control.owner:
                        inactive = isinstance(message, dict) and message.get('type') == 'command' and message.get('source') != self.control.mode
                        if not inactive:
                            self.control.stop('invalid control message')
                    reply = {'v': 1, 'type': 'error', 'message': str(exc)}
                if data['queue'].full():
                    break
                data['queue'].put_nowait(reply)
        finally:
            self.control.disconnect(ws)
            self.clients.pop(ws, None)
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task
        return ws

    async def ros_client(self, reader, writer):
        if self.bridge:
            writer.close()
            return
        self.bridge = writer
        async def send_state():
            while True:
                if self.state:
                    writer.write((dumps(self.state) + '\n').encode())
                    await asyncio.wait_for(writer.drain(), timeout=0.3)
                await asyncio.sleep(0.1)
        task = asyncio.create_task(send_state())
        task.add_done_callback(lambda _: writer.close())
        try:
            while line := await reader.readline():
                m = json.loads(line)
                if m.get('type') == 'ros_status':
                    self.ros = m
                    self.ros_at = time.monotonic()
                elif m.get('type') == 'map':
                    if m['width'] * m['height'] > self.c['ros']['max_map_cells']:
                        raise ValueError('map exceeds configured size')
                    self.map = m
        except (ValueError, KeyError, OSError):
            logging.exception('ROS bridge connection failed')
        finally:
            self.bridge = None
            self.ros_at = -float('inf')
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
            writer.close()


def main():
    from aiohttp import web
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=str(ROOT / 'config.json'))
    args = parser.parse_args()
    path = Path(args.config).resolve()
    c = json.loads(path.read_text(encoding='utf-8'))
    import fcntl
    (ROOT / 'runtime').mkdir(mode=0o700, exist_ok=True)
    lock = (ROOT / 'runtime/controller.lock').open('a')
    try:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('Controller already running; do not start a second GPIO owner')
    service = Service(c, path)
    @web.middleware
    async def lan_only(request, handler):
        if not network_allowed(c, request.remote, request.host, request.headers.get('Origin')):
            raise web.HTTPForbidden(text='trusted LAN / configured Host / same Origin only')
        response = await handler(request)
        response.headers['X-Content-Type-Options'] = 'nosniff'
        response.headers['Cache-Control'] = 'no-store'
        return response
    app = web.Application(middlewares=[lan_only])
    async def index(request):
        return web.FileResponse(ROOT / 'ui.html')
    app.add_routes([web.get('/', index), web.get('/ws', service.websocket)])
    app.on_startup.append(service.start)
    app.on_cleanup.append(service.close)
    try:
        web.run_app(app, host=c['network']['bind'], port=c['network']['port'], access_log=None)
    finally:
        service.hardware.close()


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
