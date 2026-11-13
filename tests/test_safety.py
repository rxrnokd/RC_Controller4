"""Policy unit tests with fixed inputs only; not a vehicle simulator."""
import copy
import json
import math
from pathlib import Path
import unittest

from core import Controller, Odometry, drive_errors, validate_config
from server import network_allowed
from hardware import Quadrature

ROOT = Path(__file__).resolve().parents[1]


def configured():
    c = json.loads((ROOT / 'config.json').read_text())
    # Test-only numbers. Never written into production hardware settings.
    c.update(hardware_enabled=True, wiring_verified=True)
    c['drive'].update(supply_max_v=12, pwm_limit=0.3, max_speed_m_s=0.4, open_loop_m_s_at_full_pwm=1.)
    c['servo'].update(supply_max_v=7.4, min_us=1000, center_us=1500, max_us=2000, left_rad=0.4, right_rad=0.3)
    return c


class Clock:
    def __init__(self):
        self.t = 10.
    def __call__(self):
        return self.t


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.c = configured()
        self.ctrl = Controller(self.c, self.clock)
        self.send('claim', source='MANUAL')
        self.send('reset_estop')
        self.send('arm')

    def send(self, kind, client='one', **kw):
        return self.ctrl.message(client, dict(v=1, type=kind, **kw))

    def command(self, **kw):
        msg = dict(source='MANUAL', control='pwm', throttle=1., steering_rad=0., seq=self.ctrl.last_seq+1, server_time=self.clock())
        msg.update(kw)
        self.send('command', **msg)

    def test_defaults_block_motion(self):
        c = json.loads((ROOT/'config.json').read_text())
        self.assertTrue(drive_errors(c))
        control = Controller(c, self.clock)
        control.message('x', {'v': 1, 'type': 'claim', 'source': 'MANUAL'})
        control.message('x', {'v': 1, 'type': 'reset_estop'})
        with self.assertRaises(ValueError):
            control.message('x', {'v': 1, 'type': 'arm'})

    def test_timeout_stops_and_does_not_resume(self):
        self.command()
        self.ctrl.tick(.1)
        self.assertGreater(self.ctrl.output[0], 0)
        self.clock.t += .36
        self.ctrl.tick(.02)
        self.assertEqual(self.ctrl.output, [0., 0.])
        self.assertFalse(self.ctrl.armed)
        with self.assertRaises(ValueError):
            self.command()

    def test_observer_cannot_control_but_can_estop(self):
        with self.assertRaises(ValueError):
            self.send('claim', client='two', source='MANUAL')
        with self.assertRaises(ValueError):
            self.send('limit', client='two', pwm=.1)
        self.send('estop', client='two')
        self.assertTrue(self.ctrl.estop)
        self.assertFalse(self.ctrl.armed)

    def test_disconnect_brakes(self):
        self.command()
        self.ctrl.tick(.2)
        self.ctrl.disconnect('two')
        self.assertTrue(self.ctrl.armed)
        self.ctrl.disconnect('one')
        self.assertEqual(self.ctrl.output, [0, 0])
        self.assertIsNone(self.ctrl.owner)

    def test_inactive_source_does_not_refresh_deadline(self):
        old = self.ctrl.last_command
        self.clock.t += .1
        with self.assertRaises(ValueError):
            self.command(source='AUTO_PARK')
        self.assertEqual(self.ctrl.last_command, old)

    def test_mode_switch_clears_commands(self):
        self.command()
        self.ctrl.tick(.2)
        self.send('mode', mode='MANUAL')
        self.assertFalse(self.ctrl.armed)
        self.assertEqual(self.ctrl.target['value'], 0)
        self.assertEqual(self.ctrl.output, [0, 0])

    def test_auto_unavailable_by_default(self):
        with self.assertRaises(ValueError):
            self.send('mode', mode='AUTO_PARK')
        self.assertEqual(self.ctrl.mode, 'MANUAL')
        self.assertFalse(self.ctrl.armed)

    def test_reject_nonfinite_boolean_and_replay(self):
        for value in (float('nan'), float('inf'), True, '0.2'):
            with self.assertRaises(ValueError):
                self.command(throttle=value)
        self.command(seq=10)
        with self.assertRaises(ValueError):
            self.command(seq=9)
        with self.assertRaises(ValueError):
            self.command(server_time=self.clock()-.4)
        with self.assertRaises(ValueError):
            self.command(server_time=self.clock()+1)

    def test_limit_and_ramp(self):
        self.command()
        self.ctrl.tick(.1)
        self.assertAlmostEqual(self.ctrl.output[0], .05)
        for _ in range(20):
            self.ctrl.tick(.1)
        self.assertAlmostEqual(self.ctrl.output[0], .3)
        self.send('limit', pwm=.02)
        self.assertLessEqual(abs(self.ctrl.output[0]), .02)
        with self.assertRaises(ValueError):
            self.send('limit', pwm=.31)

    def test_reversal_has_zero_and_dwell(self):
        self.command()
        self.ctrl.tick(.2)
        self.command(throttle=-1.)
        self.ctrl.tick(.2)
        self.assertEqual(self.ctrl.output[0], 0)
        self.clock.t += .05
        self.ctrl.tick(.05)
        self.assertEqual(self.ctrl.output[0], 0)
        self.clock.t += .11
        self.ctrl.tick(.05)
        self.assertLess(self.ctrl.output[0], 0)

    def test_pi_feedback_loss_latches_and_zero_clears_integrator(self):
        self.c['drive'].update(pi_enabled=True, kp=.01, ki=.01)
        self.ctrl.feedback = [{'valid': True, 'rpm': 20.} for _ in range(2)]
        self.command(control='speed', speed_m_s=.2)
        self.ctrl.tick(.1)
        self.assertGreater(self.ctrl.output[0], 0)
        self.ctrl.feedback[1] = {'valid': False, 'rpm': None}
        self.ctrl.tick(.1)
        self.assertTrue(self.ctrl.estop)
        self.assertEqual(self.ctrl.output, [0, 0])
        self.assertEqual(self.ctrl.integral, [0, 0])

    def test_servo_slew_and_missing_measured_angle(self):
        self.command(steering_rad=.4)
        self.ctrl.tick(.1)
        self.assertAlmostEqual(self.ctrl.angle, .05)
        self.assertTrue(1000 <= self.ctrl.pulse <= 2000)
        pulse = self.ctrl.pulse
        self.send('estop')
        self.ctrl.tick(.1)
        self.assertEqual(self.ctrl.pulse, pulse)

    def test_turning_speed_requires_measured_geometry(self):
        with self.assertRaises(ValueError):
            self.command(control='speed', speed_m_s=.2, steering_rad=.2)

    def test_left_turn_requests_faster_right_wheel(self):
        self.c['drive']['wheelbase_m'] = .25
        self.c['odom'].update(track_m=.2, rear_wheel_geometry_verified=True)
        self.command(control='speed', speed_m_s=.2, steering_rad=.2)
        self.ctrl.tick(.1)
        self.ctrl.tick(.1)
        self.assertLess(self.ctrl.requested[0], self.ctrl.requested[1])
        self.assertAlmostEqual(sum(self.ctrl.requested)/2, .2)

    def test_pin_conflicts_and_supply_limits(self):
        self.c['servo']['pin'] = 16
        with self.assertRaises(ValueError):
            validate_config(self.c)
        self.c['drive']['supply_max_v'] = 14
        self.assertTrue(drive_errors(self.c))

    def test_auto_requires_health_and_accepts_speed_only(self):
        self.send('release')
        self.c['auto_park'].update(algorithm_installed=True, commissioned=True)
        self.c['drive'].update(pi_enabled=True, kp=.01, ki=.01)
        self.ctrl.auto_ready = True
        self.send('claim', source='AUTO_PARK')
        self.send('algorithm_status', ready=True)
        self.send('mode', mode='AUTO_PARK')
        self.send('arm')
        with self.assertRaises(ValueError):
            self.command(source='AUTO_PARK')
        with self.assertRaises(ValueError):
            self.command(source='MANUAL')
        self.ctrl.feedback = [{'valid': True, 'rpm': 20.} for _ in range(2)]
        self.command(source='AUTO_PARK', control='speed', speed_m_s=.1)
        self.ctrl.tick(.1)
        self.ctrl.auto_ready = False
        self.ctrl.tick(.02)
        self.assertTrue(self.ctrl.estop)
        self.assertEqual(self.ctrl.output, [0, 0])


class InterfaceTests(unittest.TestCase):
    def test_lan_host_and_origin(self):
        c = configured()
        self.assertTrue(network_allowed(c, '127.0.0.1', 'localhost:8080', 'http://localhost:8080'))
        self.assertTrue(network_allowed(c, '127.0.0.1', 'localhost:8080'))
        self.assertFalse(network_allowed(c, '8.8.8.8', 'localhost:8080'))
        self.assertFalse(network_allowed(c, '127.0.0.1', 'attacker.example'))
        self.assertFalse(network_allowed(c, '127.0.0.1', 'localhost:8080', 'http://attacker.example'))

    def test_quadrature_direction_and_invalid_transition(self):
        cycle = (0, 1, 3, 2, 0)
        self.assertEqual(sum(Quadrature.TRANSITIONS[a, b] for a, b in zip(cycle, cycle[1:])), 4)
        self.assertEqual(sum(Quadrature.TRANSITIONS[b, a] for a, b in zip(cycle, cycle[1:])), -4)
        self.assertNotIn((0, 3), Quadrature.TRANSITIONS)

    def test_odom_straight_turn_and_gap(self):
        c = configured()
        c['odom'].update(enabled=True, track_m=.2)
        o = Odometry(c)
        def motors(l, r):
            return [{'valid': True, 'distance_m': l}, {'valid': True, 'distance_m': r}]
        self.assertIsNone(o.update(motors(0, 0), 1.))
        straight = o.update(motors(.1, .1), 2.)
        self.assertAlmostEqual(straight['x'], .1)
        self.assertAlmostEqual(straight['yaw'], 0)
        turn = o.update(motors(.1, .2), 3.)
        self.assertAlmostEqual(turn['yaw'], .5)
        self.assertIsNone(o.update([{'valid': False}, {'valid': False}], 4.))
        self.assertIsNone(o.update(motors(.2, .3), 5.))
        self.assertTrue(o.fault)

    def test_odom_can_use_unchanged_measured_counts_at_rest(self):
        c = configured()
        c['odom'].update(enabled=True, track_m=.2)
        o = Odometry(c)
        stationary = [{'valid': False, 'odom_valid': True, 'distance_m': 1.0} for _ in range(2)]
        self.assertIsNone(o.update(stationary, 1.))
        result = o.update(stationary, 2.)
        self.assertEqual(result['speed_m_s'], 0)
        self.assertFalse(o.fault)


if __name__ == '__main__':
    unittest.main()
