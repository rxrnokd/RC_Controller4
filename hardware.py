"""Pi OS only: pigpio DMA PWM, quadrature callbacks and sequential US-100 ranging."""
import json
import math
import os
from pathlib import Path
import select
import subprocess
import sys
import threading
import time


class Quadrature:
    # Both edges of both channels: x4 counts, not manufacturer's A-channel PPR.
    TRANSITIONS = {(0, 1): 1, (1, 3): 1, (3, 2): 1, (2, 0): 1,
                   (1, 0): -1, (3, 1): -1, (2, 3): -1, (0, 2): -1}

    def __init__(self, pi, a, b, direction=1, z=None):
        import pigpio
        self.pi, self.a, self.b, self.direction = pi, a, b, direction
        self.lock = threading.Lock()
        self.count, self.travel, self.errors, self.index = 0, 0, 0, None
        self.last_edge = None
        self.last_tick = None
        self.previous_count = 0
        self.previous_time = time.monotonic()
        self.last_rpm = None
        for pin in (a, b):
            pi.set_mode(pin, pigpio.INPUT)
            pi.set_pull_up_down(pin, pigpio.PUD_OFF)
        self.bits = (pi.read(a) << 1) | pi.read(b)
        self.callbacks = [pi.callback(p, pigpio.EITHER_EDGE, self.edge) for p in (a, b)]
        if z is not None:
            pi.set_mode(z, pigpio.INPUT)
            pi.set_pull_up_down(z, pigpio.PUD_OFF)
            self.callbacks.append(pi.callback(z, pigpio.RISING_EDGE, self.zero))

    def edge(self, gpio, level, tick):
        if level not in (0, 1):
            return
        with self.lock:
            # Use callback levels, not delayed GPIO reads, to preserve edge ordering.
            bits = (self.bits & 1) | (level << 1) if gpio == self.a else (self.bits & 2) | level
            if bits == self.bits:
                return
            delta = self.TRANSITIONS.get((self.bits, bits))
            self.bits = bits
            if delta is None:
                self.errors += 1
                return
            self.count += delta * self.direction
            self.travel += 1
            self.last_edge = time.monotonic()
            self.last_tick = tick

    def zero(self, gpio, level, tick):
        with self.lock:
            self.index = self.count

    def sample(self, cpr, timeout, diameter=None):
        now = time.monotonic()
        with self.lock:
            count, travel, index, errors, edge = self.count, self.travel, self.index, self.errors, self.last_edge
        dt = now - self.previous_time
        age = now - edge if edge is not None else None
        valid = bool(cpr and age is not None and age <= timeout and errors == 0)
        if dt >= 0.1:
            self.last_rpm = (count - self.previous_count) * 60 / (cpr * dt) if valid else None
            self.previous_count, self.previous_time = count, now
        rpm = self.last_rpm if valid else None
        return {'count': count, 'invalid_transitions': errors, 'edge_age_s': age,
                'valid': valid, 'rpm': rpm, 'turns': count / cpr if cpr else None,
                'distance_m': count / cpr * math.pi * diameter if cpr and diameter else None,
                'travel_m': travel / cpr * math.pi * diameter if cpr and diameter else None,
                'angle_rad': (count % cpr) / cpr * 2 * math.pi if cpr else None,
                'index_angle_rad': ((count-index) % cpr) / cpr * 2 * math.pi if cpr and index is not None else None,
                'index_seen': index is not None}

    def close(self):
        for cb in self.callbacks:
            cb.cancel()


class Hardware:
    def __init__(self, c, config_path):
        self.c, self.enabled = c, c['hardware_enabled']
        self.reason = 'hardware disabled; no measurements'
        self.guard = None
        self.pi = None
        self.encoders = []
        self.mt = None
        self.sensor_thread = None
        self.done = threading.Event()
        self.ultras = [{'cm': None, 'status': 'disabled', 'measured_at': None} for _ in range(4)]
        if not self.enabled:
            return
        if not c['wiring_verified']:
            raise ValueError('verify 3.3V GPIO wiring and pin conflicts first')
        import pigpio
        self.pi = pigpio.pi('127.0.0.1')
        if not self.pi.connected:
            raise RuntimeError('pigpiod not connected')
        try:
            self.brake()
            for m in c['drive']['motors']:
                for pin in m['pwm']:
                    self.pi.set_mode(pin, pigpio.OUTPUT)
                    self.pi.set_PWM_range(pin, 10000)
                    actual = self.pi.set_PWM_frequency(pin, c['drive']['pwm_hz'])
                    if not 0 < actual <= 20000:
                        raise ValueError('unsupported PWM frequency')
                    m['actual_pwm_hz'] = actual
                self.encoders.append(Quadrature(self.pi, *m['encoder'], m['encoder_direction']))
            self.pi.set_servo_pulsewidth(c['servo']['pin'], 0)
            t = c['mt6701']
            self.mt = Quadrature(self.pi, t['a'], t['b'], t['direction'], t['z'])
            self.guard = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--guard', str(config_path)],
                                          stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
            # Guard acknowledges only after it has connected and applied zero outputs.
            if not select.select([self.guard.stdout], [], [], 3)[0] or self.guard.stdout.readline() != b'READY\n':
                raise RuntimeError('output watchdog failed to start')
            if c['ultrasonic']['enabled']:
                self.sensor_thread = threading.Thread(target=self.measure_loop, daemon=True)
                self.sensor_thread.start()
            self.reason = 'GPIO connected; signals still require physical validation'
        except Exception:
            self.close()
            raise

    def brake(self):
        if self.pi is not None and self.pi.connected:
            for m in self.c['drive']['motors']:
                for pin in m['pwm']:
                    self.pi.set_PWM_dutycycle(pin, 0)

    def apply(self, control):
        if not self.enabled:
            return
        if not self.pi.connected or self.guard.poll() is not None:
            self.brake()
            raise RuntimeError('GPIO / watchdog failure; restart required')
        # Heartbeat only from the control loop. A stuck web server cannot keep it alive.
        self.guard.stdin.write(b'.')
        for i, m in enumerate(self.c['drive']['motors']):
            pwm = control.output[i] * m['direction']
            a, b = m['pwm']
            # Clear inactive input before enabling active input (MDD3A A/B, not DIR/EN).
            inactive, active = (b, a) if pwm >= 0 else (a, b)
            self.pi.set_PWM_dutycycle(inactive, 0)
            self.pi.set_PWM_dutycycle(active, round(abs(pwm) * 10000))
        if control.pulse is not None:
            self.pi.set_servo_pulsewidth(self.c['servo']['pin'], control.pulse)

    def measure_loop(self):
        import pigpio
        u = self.c['ultrasonic']
        try:
            for trig, echo in u['pins']:
                self.pi.set_mode(trig, pigpio.OUTPUT)
                self.pi.write(trig, 0)
                self.pi.set_mode(echo, pigpio.INPUT)
            while not self.done.is_set():
                for i, (trig, echo) in enumerate(u['pins']):
                    start = time.monotonic()
                    result = {'cm': None, 'status': 'timeout', 'measured_at': start}
                    event, ticks = threading.Event(), []
                    def edge(gpio, level, tick):
                        if level == 1 and not ticks:
                            ticks.append(tick)
                        elif level == 0 and len(ticks) == 1:
                            ticks.append(tick)
                            event.set()
                    cb = self.pi.callback(echo, pigpio.EITHER_EDGE, edge)
                    try:
                        if self.pi.read(echo):
                            result['status'] = 'echo_stuck_high'
                        else:
                            self.pi.gpio_trigger(trig, 10, 1)
                            if event.wait(u['timeout_s']):
                                cm = pigpio.tickDiff(ticks[0], ticks[1]) * 0.0343 / 2
                                result['status'] = 'ok' if u['min_cm'] <= cm <= u['max_cm'] else 'out_of_range'
                                if result['status'] == 'ok':
                                    result['cm'] = round(cm, 2)
                    finally:
                        cb.cancel()
                    self.ultras[i] = result
                    if self.done.wait(max(0, u['gap_s'] - (time.monotonic() - start))):
                        return
        except Exception as exc:
            self.ultras = [{'cm': None, 'status': str(exc), 'measured_at': None} for _ in range(4)]

    def sample(self):
        timeout, d = self.c['safety']['feedback_timeout_s'], self.c['drive']
        if not self.enabled:
            motors = [{'valid': False, 'rpm': None, 'turns': None, 'distance_m': None, 'travel_m': None, 'count': None} for _ in range(2)]
            mt = {'angle_rad': None, 'index_angle_rad': None, 'count': None, 'index_seen': False, 'valid': False}
        else:
            motors = [e.sample(m['counts_per_wheel_rev'], timeout, d['wheel_diameter_m']) for e, m in zip(self.encoders, d['motors'])]
            mt = self.mt.sample(self.c['mt6701']['counts_per_rev'], timeout)
            if mt['index_angle_rad'] is not None:
                mt['index_angle_rad'] = (mt['index_angle_rad'] + self.c['mt6701']['index_offset_rad']) % (2 * math.pi)
        # Independent trailer-axis sensor, not a drive wheel or steering sensor.
        # Preserve the existing protocol key while excluding motor-only metrics.
        for key in ('rpm', 'turns', 'distance_m', 'travel_m'):
            mt.pop(key, None)
        mt['role'] = 'trailer_axis_angle'
        ultras = []
        for value in self.ultras:
            value = dict(value)
            value['age_s'] = time.monotonic() - value['measured_at'] if value['measured_at'] is not None else None
            if value['age_s'] is not None and value['age_s'] > 0.6:
                value.update(cm=None, status='stale')
            ultras.append(value)
        return {'motors': motors, 'mt6701': mt, 'ultrasonic': ultras, 'hardware': self.reason}

    def close(self):
        self.done.set()
        if self.sensor_thread:
            self.sensor_thread.join(timeout=0.3)
        try:
            self.brake()
            if self.pi and self.pi.connected:
                self.pi.set_servo_pulsewidth(self.c['servo']['pin'], 0)
        finally:
            if self.guard:
                guard, self.guard = self.guard, None
                guard.stdin.close()
                guard.wait(timeout=2)
            for e in self.encoders + ([self.mt] if self.mt else []):
                e.close()
            if self.pi:
                self.pi.stop()
                self.pi = None


def watchdog(config_path):
    """Independent process: EOF/main kill or missed heartbeat => latched brake + servo off."""
    import pigpio
    c = json.loads(Path(config_path).read_text())
    pi = pigpio.pi('127.0.0.1')
    if not pi.connected:
        raise RuntimeError('watchdog GPIO unavailable')
    def off():
        for m in c['drive']['motors']:
            for pin in m['pwm']:
                pi.set_PWM_dutycycle(pin, 0)
        pi.set_servo_pulsewidth(c['servo']['pin'], 0)
    try:
        off()
        print('READY', flush=True)
        while select.select([sys.stdin.buffer], [], [], c['safety']['guard_timeout_s'])[0]:
            if not os.read(sys.stdin.fileno(), 4096):
                break
    finally:
        off()
        pi.stop()


if __name__ == '__main__' and len(sys.argv) == 3 and sys.argv[1] == '--guard':
    watchdog(sys.argv[2])
