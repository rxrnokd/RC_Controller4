"""Pure control policy; no fabricated hardware values and no GPIO dependencies."""
import math
import time


def number(value, name, low=-math.inf, high=math.inf):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name}: finite number in [{low}, {high}] required")
    return float(value)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def ramp(current, target, step):
    return current + clamp(target - current, -step, step)


def drive_errors(c):
    errors = []
    d, s = c['drive'], c['servo']
    if not c['hardware_enabled'] or not c['wiring_verified']:
        errors.append('hardware_enabled / wiring_verified required')
    checks = [(d['supply_max_v'], 'motor supply_max_v', 4, 12),
              (d['pwm_limit'], 'pwm_limit', 0.001, 1),
              (s['supply_max_v'], 'servo supply_max_v', 6, 8.4),
              (s['min_us'], 'servo min_us', 500, 2500),
              (s['center_us'], 'servo center_us', 500, 2500),
              (s['max_us'], 'servo max_us', 500, 2500),
              (s['left_rad'], 'left road wheel angle', 0.001, 1.5),
              (s['right_rad'], 'right road wheel angle', 0.001, 1.5)]
    for args in checks:
        try:
            number(*args)
        except ValueError as e:
            errors.append(str(e))
    if all(s[k] is not None for k in ('min_us', 'center_us', 'max_us')):
        if not s['min_us'] < s['center_us'] < s['max_us']:
            errors.append('servo min_us < center_us < max_us required')
    return errors


def validate_config(c):
    def finite_tree(value):
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError('non-finite configuration value')
        if isinstance(value, dict):
            for item in value.values():
                finite_tree(item)
        elif isinstance(value, list):
            for item in value:
                finite_tree(item)
    finite_tree(c)
    pins = [c['servo']['pin'], *[c['mt6701'][k] for k in ('a', 'b', 'z')]]
    for m in c['drive']['motors']:
        pins += m['pwm'] + m['encoder']
        for key in ('direction', 'encoder_direction'):
            if m[key] not in (-1, 1):
                raise ValueError(key)
        if m['counts_per_wheel_rev'] is not None:
            number(m['counts_per_wheel_rev'], 'counts_per_wheel_rev', 1)
    for pair in c['ultrasonic']['pins']:
        pins += pair
    if len(pins) != len(set(pins)) or any(type(p) is not int or not 0 <= p <= 27 for p in pins):
        raise ValueError('BCM GPIO conflict/range')
    if len(c['drive']['motors']) != 2 or len(c['ultrasonic']['pins']) != 4:
        raise ValueError('two motors and four ultrasound sensors required')
    number(c['drive']['pwm_hz'], 'PWM frequency', 1, 20000)
    number(c['drive']['wheel_diameter_m'], 'wheel diameter', 0.001, 1)
    for key, value in c['safety'].items():
        number(value, key, 0.001, 5)
    if c['servo']['direction'] not in (-1, 1) or c['mt6701']['direction'] not in (-1, 1):
        raise ValueError('direction must be +/-1')
    if c['mt6701']['counts_per_rev'] is not None:
        number(c['mt6701']['counts_per_rev'], 'MT6701 x4 counts/rev', 1)
    if c['ultrasonic']['enabled']:
        u = c['ultrasonic']
        number(u['min_cm'], 'ultrasonic min_cm', 0)
        number(u['max_cm'], 'ultrasonic max_cm', u['min_cm'] + 0.01)
    number(c['ultrasonic']['timeout_s'], 'echo timeout', 0.001, 0.1)
    number(c['ultrasonic']['gap_s'], 'echo gap', 0.06, 1)
    if c['drive']['pi_enabled']:
        number(c['drive']['kp'], 'kp', 0)
        number(c['drive']['ki'], 'ki', 0)
    for key in ('max_speed_m_s', 'open_loop_m_s_at_full_pwm', 'wheelbase_m'):
        if c['drive'][key] is not None:
            number(c['drive'][key], key, 0.001)
    laser = c['ros']['laser_xyz_yaw']
    if laser is not None:
        if not isinstance(laser, list) or len(laser) != 4:
            raise ValueError('laser_xyz_yaw requires [x,y,z,yaw]')
        for value in laser:
            number(value, 'laser transform')
    number(c['ros']['freshness_s'], 'ROS freshness', .1, 5)
    number(c['ros']['max_map_cells'], 'map size limit', 1, 2000000)
    if c['auto_park']['obstacle_stop_cm'] is not None:
        number(c['auto_park']['obstacle_stop_cm'], 'obstacle stop cm', 0.1)
    if c['odom']['enabled']:
        o = c['odom']
        number(o['track_m'], 'track_m', 0.01, 2)
        if not o['rear_wheel_geometry_verified'] or {o['left_motor'], o['right_motor']} != {0, 1}:
            raise ValueError('verified rear wheel geometry required')
        if any(m['counts_per_wheel_rev'] is None for m in c['drive']['motors']):
            raise ValueError('odom requires both encoder calibrations')


class Controller:
    def __init__(self, config, clock=time.monotonic):
        validate_config(config)
        self.c, self.clock = config, clock
        self.mode, self.owner, self.role = 'MANUAL', None, None
        self.estop, self.armed = True, False
        self.reason = 'startup: emergency stop latched'
        self.last_command, self.last_seq = 0, -1
        self.algorithm_at = -math.inf
        self.feedback = [None, None]
        self.auto_ready = False
        self.target = {'kind': 'pwm', 'value': 0.0, 'steering_rad': 0.0}
        self.output, self.requested, self.integral = [0., 0.], [0., 0.], [0., 0.]
        self.angle, self.pulse = 0., None
        self.limit = config['drive']['pwm_limit'] or 0.
        self.reverse_until = [0., 0.]
        self.last_direction = [0, 0]

    def stop(self, reason, latch=False):
        self.target = {'kind': 'pwm', 'value': 0., 'steering_rad': 0.}
        self.output = [0., 0.]
        self.requested = [0., 0.]
        self.integral = [0., 0.]
        self.armed = False
        self.reason = reason
        if latch:
            self.estop = True
        # Servo holds its last commanded position; never jumps to center on stop.

    def disconnect(self, client):
        if client == self.owner:
            self.stop('owner disconnected')
            self.owner = self.role = None
            self.algorithm_at = -math.inf

    def message(self, client, m):
        if not isinstance(m, dict) or type(m.get('v')) is not int or m.get('v') != 1:
            raise ValueError('protocol v=1 required')
        kind = m.get('type')
        if kind == 'estop':
            self.stop('emergency stop', True)
            return
        if kind == 'claim':
            if self.owner not in (None, client):
                raise ValueError('another controller owns the vehicle')
            if m.get('source') not in ('MANUAL', 'AUTO_PARK'):
                raise ValueError('invalid source')
            if self.owner == client:
                raise ValueError('already claimed; release before changing role')
            self.owner, self.role = client, m['source']
            self.last_seq = -1
            self.stop('control claimed')
            return
        if client != self.owner:
            raise ValueError('claim control first')
        if kind == 'release':
            self.disconnect(client)
        elif kind == 'stop':
            self.stop('stop command')
        elif kind == 'reset_estop':
            self.stop('emergency stop reset; disarmed')
            self.estop = False
        elif kind == 'algorithm_status':
            if self.role != 'AUTO_PARK' or m.get('ready') is not True:
                self.algorithm_at = -math.inf
                self.stop('algorithm unavailable')
            else:
                self.algorithm_at = self.clock()
        elif kind == 'mode':
            mode = m.get('mode')
            if mode not in ('MANUAL', 'AUTO_PARK'):
                raise ValueError('invalid mode')
            self.stop('mode change: commands cleared')
            if mode == 'AUTO_PARK' and not self.can_auto():
                raise ValueError('AUTO_PARK prerequisites unavailable')
            self.mode = mode
        elif kind == 'limit':
            self.limit = number(m.get('pwm'), 'pwm limit', 0, self.c['drive']['pwm_limit'] or 0)
            self.output = [clamp(x, -self.limit, self.limit) for x in self.output]
            self.integral = [0., 0.]
        elif kind == 'arm':
            errors = drive_errors(self.c)
            if errors or self.estop or self.role != self.mode:
                raise ValueError('; '.join(errors) or 'estop or source/mode mismatch')
            if self.mode == 'AUTO_PARK' and not self.can_auto():
                raise ValueError('AUTO_PARK not ready')
            self.stop('armed, waiting for command')
            self.armed = True
            self.last_command = self.clock()
        elif kind == 'command':
            # Inactive source cannot affect either outputs or command timeout.
            if m.get('source') != self.mode or self.role != self.mode:
                raise ValueError('inactive command source ignored')
            if not self.armed or self.estop:
                raise ValueError('not armed')
            seq = m.get('seq')
            if type(seq) is not int or seq <= self.last_seq:
                raise ValueError('sequence must increase')
            age = self.clock() - number(m.get('server_time'), 'server_time', 0)
            if not 0 <= age <= self.c['safety']['command_timeout_s']:
                raise ValueError('stale command: echo latest state server_time')
            s = self.c['servo']
            angle = number(m.get('steering_rad'), 'steering_rad', -s['right_rad'], s['left_rad'])
            control = m.get('control')
            if control == 'pwm' and self.mode == 'MANUAL':
                value = number(m.get('throttle'), 'throttle', -1, 1)
            elif control == 'speed':
                value = number(m.get('speed_m_s'), 'speed_m_s')
                maximum = number(self.c['drive']['max_speed_m_s'], 'configured max speed', 0.001)
                value = clamp(value, -maximum, maximum)
                if self.mode == 'AUTO_PARK' and not self.c['drive']['pi_enabled']:
                    raise ValueError('AUTO_PARK requires PI feedback')
                if not self.c['drive']['pi_enabled']:
                    number(self.c['drive']['open_loop_m_s_at_full_pwm'], 'calibrated open loop scale', 0.001)
                if value and (angle or self.angle):
                    number(self.c['drive']['wheelbase_m'], 'wheelbase for turning speed control', 0.01)
                    number(self.c['odom']['track_m'], 'rear track for turning speed control', 0.01)
                    if not self.c['odom']['rear_wheel_geometry_verified']:
                        raise ValueError('turning speed control requires verified rear wheel geometry')
                    if {self.c['odom']['left_motor'], self.c['odom']['right_motor']} != {0, 1}:
                        raise ValueError('left/right motor indices must be 0 and 1')
            else:
                raise ValueError('invalid control; AUTO_PARK accepts speed only')
            self.target = {'kind': control, 'value': value, 'steering_rad': angle}
            self.last_seq, self.last_command = seq, self.clock()
        else:
            raise ValueError('unknown message type')

    def can_auto(self):
        a = self.c['auto_park']
        return bool(a['algorithm_installed'] and a['commissioned'] and self.auto_ready and
                    self.c['drive']['pi_enabled'] and self.clock() - self.algorithm_at < 0.5)

    def tick(self, dt):
        now, d, s = self.clock(), self.c['drive'], self.c['servo']
        if self.armed and now - self.last_command > self.c['safety']['command_timeout_s']:
            self.stop('command timeout')
        if self.armed and self.mode == 'AUTO_PARK' and not self.can_auto():
            self.stop('AUTO_PARK health lost', True)
        if not self.armed or self.estop:
            return
        value = self.target['value']
        for i in range(2):
            wheel_value = value
            if self.target['kind'] == 'speed' and value and self.angle:
                o = self.c['odom']
                correction = o['track_m'] * math.tan(self.angle) / (2 * d['wheelbase_m'])
                wheel_value *= 1 - correction if i == o['left_motor'] else 1 + correction
            if self.target['kind'] == 'speed' and d['pi_enabled'] and value != 0:
                f = self.feedback[i]
                if not f or f['rpm'] is None or not f['valid']:
                    self.stop('RPM feedback lost: explicit re-arm required', True)
                    return
                target_rpm = wheel_value * 60 / (math.pi * d['wheel_diameter_m'])
                err = target_rpm - f['rpm']
                proposed = self.integral[i] + d['ki'] * err * dt
                raw = d['kp'] * err + proposed
                if abs(raw) <= self.limit or raw * err < 0:
                    self.integral[i] = proposed
                pwm = clamp(d['kp'] * err + self.integral[i], -self.limit, self.limit)
            else:
                self.integral[i] = 0.
                pwm = value if self.target['kind'] == 'pwm' else (wheel_value / d['open_loop_m_s_at_full_pwm'] if value else 0.)
                pwm = clamp(pwm, -self.limit, self.limit)
            self.requested[i] = pwm
            direction = 1 if pwm > 0 else -1 if pwm < 0 else 0
            reversing = direction and self.last_direction[i] and direction != self.last_direction[i]
            target = 0. if reversing else pwm
            self.output[i] = ramp(self.output[i], target, self.c['safety']['pwm_slew_per_s'] * dt)
            if reversing and self.output[i] == 0:
                self.reverse_until[i] = now + self.c['safety']['reverse_pause_s']
                self.last_direction[i] = direction
            if now < self.reverse_until[i]:
                self.output[i] = 0.
            elif not reversing and direction:
                self.last_direction[i] = direction
        self.angle = ramp(self.angle, self.target['steering_rad'], self.c['safety']['steering_slew_rad_s'] * dt)
        a = self.angle
        fraction = a / (s['left_rad'] if a >= 0 else s['right_rad']) * s['direction']
        self.pulse = round(s['center_us'] + fraction * ((s['max_us'] - s['center_us']) if fraction >= 0 else (s['center_us'] - s['min_us'])))
        self.pulse = clamp(self.pulse, s['min_us'], s['max_us'])


class Odometry:
    """Measured rear left/right wheel integration; gaps invalidate pose continuity."""
    def __init__(self, c):
        self.c = c
        self.previous = None
        self.pose = [0., 0., 0.]
        self.fault = False

    def update(self, motors, now):
        o = self.c['odom']
        valid = o['enabled'] and all(m.get('odom_valid', m.get('valid')) and m.get('distance_m') is not None for m in motors)
        if not valid:
            if self.previous is not None:
                self.fault = True
            return None
        if self.fault:
            return None
        distances = [motors[o[k]]['distance_m'] for k in ('left_motor', 'right_motor')]
        if self.previous is None:
            self.previous = (*distances, now)
            return None
        dl, dr = distances[0] - self.previous[0], distances[1] - self.previous[1]
        dt = now - self.previous[2]
        self.previous = (*distances, now)
        if dt <= 0:
            return None
        ds, dyaw = (dl + dr) / 2, (dr - dl) / o['track_m']
        x, y, yaw = self.pose
        sinc = math.sin(dyaw / 2) / (dyaw / 2) if abs(dyaw) > 1e-8 else 1.
        self.pose = [x + ds * sinc * math.cos(yaw + dyaw / 2), y + ds * sinc * math.sin(yaw + dyaw / 2), yaw + dyaw]
        return dict(zip(('x', 'y', 'yaw'), self.pose), speed_m_s=ds/dt, yaw_rate=dyaw/dt, valid=True)
