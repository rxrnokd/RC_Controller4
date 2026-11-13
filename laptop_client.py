"""Future laptop algorithm adapter; no planner/MPC and no direct AUTO PWM API."""
import asyncio
import time
import aiohttp


class VehicleClient:
    def __init__(self, url):
        self.url = url.rstrip('/') + '/ws'
        self.state = self.map = None
        self.received_at = 0.
        self.seq = 0
        self.responses = asyncio.Queue()

    async def __aenter__(self):
        self.session = aiohttp.ClientSession()
        try:
            self.ws = await self.session.ws_connect(self.url, max_msg_size=16*1024*1024, heartbeat=1)
        except Exception:
            await self.session.close()
            raise
        self.reader = asyncio.create_task(self.receive())
        return self

    async def receive(self):
        async for event in self.ws:
            if event.type != aiohttp.WSMsgType.TEXT:
                continue
            m = event.json()
            if m['type'] == 'state':
                self.state, self.received_at = m, time.monotonic()
            elif m['type'] == 'map':
                self.map = m
            elif m['type'] in ('ack', 'error'):
                await self.responses.put(m)

    async def send(self, message):
        await self.ws.send_json(dict(v=1, **message))

    async def request(self, message):
        # Calls must be serialized; errors are surfaced instead of silently arming.
        await self.send(message)
        reply = await asyncio.wait_for(self.responses.get(), 1)
        if reply['type'] == 'error':
            raise RuntimeError(reply['message'])
        return reply

    async def claim(self):
        await self.request({'type': 'claim', 'source': 'AUTO_PARK'})

    async def algorithm_ready(self, ready):
        # Call only after planner, localization checks and sensor checks succeed.
        await self.request({'type': 'algorithm_status', 'ready': bool(ready)})

    async def activate(self):
        await self.request({'type': 'mode', 'mode': 'AUTO_PARK'})
        await self.request({'type': 'reset_estop'})
        await self.request({'type': 'arm'})

    async def drive(self, speed_m_s, steering_rad):
        if not self.state or time.monotonic()-self.received_at > 0.2:
            await self.stop()
            raise RuntimeError('vehicle state stale')
        self.seq += 1
        await self.request({'type': 'command', 'source': 'AUTO_PARK', 'seq': self.seq,
                            'server_time': self.state['server_time'], 'control': 'speed',
                            'speed_m_s': speed_m_s, 'steering_rad': steering_rad})

    async def stop(self):
        await self.send({'type': 'stop'})

    async def __aexit__(self, *exc):
        try:
            if not self.ws.closed:
                await self.stop()
                await self.send({'type': 'release'})
        finally:
            await self.ws.close()
            self.reader.cancel()
            await asyncio.gather(self.reader, return_exceptions=True)
            await self.session.close()


async def monitor(url):
    # Read-only monitor, never claims control, enables AUTO, or invents commands.
    async with VehicleClient(url) as vehicle:
        while True:
            await asyncio.sleep(1)
            if vehicle.state:
                s = vehicle.state
                print(s['mode'], s['reason'], s['ros'])


if __name__ == '__main__':
    import sys
    asyncio.run(monitor(sys.argv[1]))
