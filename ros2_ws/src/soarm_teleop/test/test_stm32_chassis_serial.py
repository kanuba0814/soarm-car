#!/usr/bin/env python3

import importlib
import importlib.util
import struct
import sys
import unittest
from pathlib import Path

PACKAGE_DIR = Path(__file__).parents[1] / 'soarm_teleop'

if 'soarm_teleop' not in sys.modules:
    spec = importlib.util.spec_from_file_location(
        'soarm_teleop', PACKAGE_DIR / '__init__.py',
        submodule_search_locations=[str(PACKAGE_DIR)])
    package = importlib.util.module_from_spec(spec)
    sys.modules['soarm_teleop'] = package
    spec.loader.exec_module(package)

M = importlib.import_module('soarm_teleop.stm32_chassis_serial')
P = importlib.import_module('soarm_teleop.chassis_protocol')

T0 = 100.0


def make_policy(**kwargs):
    kwargs.setdefault('command_timeout', 0.5)
    return M.ChassisLinkPolicy(**kwargs)


class ChassisLinkPolicyTest(unittest.TestCase):

    def test_enter_and_release_frame_order(self):
        policy = make_policy()
        self.assertEqual(policy.enter_pc_mode_frames(),
                         [P.encode_stop(), P.encode_set_mode(P.MODE_PC)])
        self.assertEqual(policy.release_frames(),
                         [P.encode_stop(), P.encode_set_mode(P.MODE_REMOTE)])

    def test_fresh_command_streams_velocity_frames(self):
        policy = make_policy()
        policy.update_command(0.1, -0.05, 0.25, T0)
        frames = policy.tick(T0 + 0.1)
        self.assertEqual(frames, [P.encode_set_velocity(100.0, -50.0, 250.0)])
        self.assertEqual(policy.tick(T0 + 0.15), [P.encode_set_velocity(100.0, -50.0, 250.0)])

    def test_units_convert_mps_to_mm_per_s(self):
        policy = make_policy()
        policy.update_command(0.123, 0.0, -1.5, T0)
        frames = policy.tick(T0)
        self.assertEqual(frames, [P.encode_set_velocity(123.0, 0.0, -1500.0)])

    def test_wz_scale_applies_before_unit_conversion(self):
        policy = make_policy(wz_scale=0.5)
        policy.update_command(0.0, 0.0, 2.0, T0)
        self.assertEqual(policy.tick(T0), [P.encode_set_velocity(0.0, 0.0, 1000.0)])

    def test_commands_clamped_to_configured_limits(self):
        policy = make_policy(max_linear=0.5, max_angular=2.0)
        policy.update_command(5.0, -5.0, 99.0, T0)
        self.assertEqual(policy.tick(T0), [P.encode_set_velocity(500.0, -500.0, 2000.0)])

    def test_nonfinite_command_components_sent_as_zero(self):
        policy = make_policy()
        policy.update_command(float('nan'), float('inf'), float('nan'), T0)
        self.assertEqual(policy.tick(T0), [P.encode_set_velocity(0.0, 0.0, 0.0)])

    def test_stale_command_stops_once_then_goes_silent(self):
        policy = make_policy(command_timeout=0.5)
        policy.update_command(0.1, 0.0, 0.0, T0)
        self.assertEqual(policy.tick(T0 + 0.4), [P.encode_set_velocity(100.0, 0.0, 0.0)])
        # One stop frame when the command first goes stale...
        self.assertEqual(policy.tick(T0 + 0.6), [P.encode_stop()])
        # ...then nothing: the firmware 500 ms watchdog stays an honest fail-safe.
        self.assertEqual(policy.tick(T0 + 0.65), [])
        self.assertEqual(policy.tick(T0 + 0.7), [])

    def test_never_commanded_means_single_stop_then_silent(self):
        policy = make_policy()
        self.assertEqual(policy.tick(T0), [P.encode_stop()])
        self.assertEqual(policy.tick(T0 + 0.05), [])

    def test_velocity_resumes_after_stale_without_extra_stop(self):
        policy = make_policy(command_timeout=0.5)
        policy.update_command(0.1, 0.0, 0.0, T0)
        self.assertEqual(policy.tick(T0 + 0.6), [P.encode_stop()])
        policy.update_command(-0.1, 0.0, 0.0, T0 + 0.7)
        self.assertEqual(policy.tick(T0 + 0.75), [P.encode_set_velocity(-100.0, 0.0, 0.0)])

    def test_status_poll_appends_at_requested_period(self):
        policy = make_policy(status_poll_period=0.2)
        policy.update_command(0.0, 0.0, 0.0, T0)
        self.assertEqual(policy.tick(T0),
                         [P.encode_set_velocity(0.0, 0.0, 0.0), P.encode_get_status()])
        self.assertEqual(policy.tick(T0 + 0.1), [P.encode_set_velocity(0.0, 0.0, 0.0)])
        self.assertEqual(policy.tick(T0 + 0.2),
                         [P.encode_set_velocity(0.0, 0.0, 0.0), P.encode_get_status()])

    def test_status_poll_minimum_period_enforced(self):
        with self.assertRaises(ValueError):
            make_policy(status_poll_period=0.05)

    def test_reset_clears_command_state(self):
        policy = make_policy()
        policy.update_command(0.1, 0.0, 0.0, T0)
        policy.tick(T0)
        policy.reset()
        self.assertIsNone(policy.command)
        self.assertEqual(policy.tick(T0 + 0.01), [P.encode_stop()])

    def test_invalid_configuration_rejected(self):
        with self.assertRaises(ValueError):
            make_policy(command_timeout=0.0)
        with self.assertRaises(ValueError):
            make_policy(wz_scale=0.0)
        with self.assertRaises(ValueError):
            make_policy(max_linear=-1.0)


class ChassisLinkRxTest(unittest.TestCase):

    def test_feed_rx_decodes_status_and_move_acks(self):
        policy = make_policy()
        payload = bytearray(52)
        struct.pack_into('<BBH', payload, 16, P.MODE_PC, 0, 0)
        struct.pack_into('<4f', payload, 20, 1.0, 1.0, 1.0, 1.0)
        status_frame = P.encode_frame(P.CMD_STATUS_ACK, bytes(payload))
        move_frame = P.encode_frame(P.CMD_MOVE_ACK, struct.pack('<ffB', 10.0, 20.0, 0))
        acks = policy.feed_rx(status_frame + move_frame)
        self.assertEqual(acks, [
            ('status', {'mode': P.MODE_PC, 'w_target': [1.0] * 4, 'w_actual': [0.0] * 4}),
            ('move', {'odom_x_mm': 10.0, 'odom_y_mm': 20.0, 'queue_left': 0}),
        ])
        self.assertEqual(policy.rx_buffer, b'')

    def test_feed_rx_buffers_partial_frames(self):
        policy = make_policy()
        whole = P.encode_frame(P.CMD_MOVE_ACK, struct.pack('<ffB', 1.0, 2.0, 3))
        self.assertEqual(policy.feed_rx(whole[:5]), [])
        self.assertEqual(policy.feed_rx(whole[5:]),
                         [('move', {'odom_x_mm': 1.0, 'odom_y_mm': 2.0, 'queue_left': 3})])

    def test_feed_rx_ignores_garbage_and_unknown_frames(self):
        policy = make_policy()
        self.assertEqual(policy.feed_rx(b'noise' + P.encode_heartbeat()), [])
        self.assertEqual(policy.feed_rx(P.encode_stop()), [])

    def test_rx_buffer_stays_bounded(self):
        policy = make_policy()
        policy.feed_rx(b'\x00' * 8192)
        self.assertLessEqual(len(policy.rx_buffer), M.MAX_RX_BUFFER)


if __name__ == '__main__':
    unittest.main()
