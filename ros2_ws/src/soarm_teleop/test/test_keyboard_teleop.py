#!/usr/bin/env python3

import importlib
import importlib.util
import math
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

M = importlib.import_module('soarm_teleop.keyboard_teleop')

T0 = 100.0


class TeleopPolicyTest(unittest.TestCase):

    def make_policy(self, **kwargs):
        return M.TeleopPolicy(max_linear=0.4, max_angular=1.0, **kwargs)

    def press(self, policy, keys, now=T0):
        for key in keys:
            self.assertTrue(policy.handle_key(key, now))

    def test_forward_key_gives_forward_command(self):
        policy = self.make_policy()
        self.press(policy, 'i')
        self.assertAlmostEqual(policy.command(T0)[0], 0.4)
        self.assertAlmostEqual(policy.command(T0)[1], 0.0)
        self.assertAlmostEqual(policy.command(T0)[2], 0.0)

    def test_strafe_and_back_keys(self):
        policy = self.make_policy()
        for key, axis, expected in (('j', 1, 0.4), (',', 0, -0.4), ('l', 1, -0.4)):
            policy.handle_key('k', T0)  # held keys accumulate: reset between probes
            self.press(policy, key)
            self.assertAlmostEqual(policy.command(T0)[axis], expected)

    def test_rotate_keys(self):
        policy = self.make_policy()
        policy.handle_key('k', T0)
        self.press(policy, 'q')
        self.assertAlmostEqual(policy.command(T0)[2], 1.0)
        policy.handle_key('k', T0)
        self.press(policy, 'e')
        self.assertAlmostEqual(policy.command(T0)[2], -1.0)

    def test_diagonal_keys_are_normalized(self):
        policy = self.make_policy()
        self.press(policy, 'u')
        vx, vy, _ = policy.command(T0)
        self.assertAlmostEqual(math.hypot(vx, vy), 0.4)
        self.assertAlmostEqual(vx, 0.4 / math.sqrt(2))
        self.assertAlmostEqual(vy, 0.4 / math.sqrt(2))

    def test_translate_plus_tapped_rotate_combines(self):
        policy = self.make_policy()
        self.press(policy, 'i')
        self.press(policy, 'q', now=T0 + 0.1)
        vx, vy, wz = policy.command(T0 + 0.2)
        self.assertAlmostEqual(vx, 0.4)
        self.assertAlmostEqual(wz, 1.0)

    def test_opposite_keys_cancel(self):
        policy = self.make_policy()
        self.press(policy, ['i', ','])
        command = policy.command(T0)
        self.assertEqual(command, (0.0, 0.0, 0.0))

    def test_stop_key_clears_motion(self):
        policy = self.make_policy()
        self.press(policy, ['i', 'q'])
        self.press(policy, ['k'])
        self.assertEqual(policy.command(T0), (0.0, 0.0, 0.0))
        self.press(policy, ['i'])
        self.press(policy, [' '])
        self.assertEqual(policy.command(T0), (0.0, 0.0, 0.0))

    def test_keys_expire_after_hold_timeout(self):
        policy = self.make_policy(hold_timeout=1.0)
        self.press(policy, 'i', now=T0)
        self.assertAlmostEqual(policy.command(T0 + 0.99)[0], 0.4)
        self.assertEqual(policy.command(T0 + 1.0), (0.0, 0.0, 0.0))
        self.assertEqual(policy.active_keys(T0 + 1.0), [])

    def test_rotation_and_translation_expire_independently(self):
        policy = self.make_policy(hold_timeout=1.0)
        self.press(policy, 'i', now=T0)
        self.press(policy, 'q', now=T0 + 0.6)
        vx, _, wz = policy.command(T0 + 1.3)
        self.assertAlmostEqual(vx, 0.0)  # forward expired at T0 + 1.0
        self.assertAlmostEqual(wz, 1.0)  # rotate still fresh until T0 + 1.6

    def test_speed_keys_step_and_clamp(self):
        policy = self.make_policy()
        self.assertEqual(policy.scale, 1.0)
        for _ in range(5):
            self.assertTrue(policy.handle_key('z', T0))
        self.assertAlmostEqual(policy.scale, 0.5)
        self.press(policy, 'i')
        self.assertAlmostEqual(policy.command(T0)[0], 0.2)
        for _ in range(20):
            policy.handle_key('z', T0)
        self.assertAlmostEqual(policy.scale, 0.1)
        for _ in range(20):
            policy.handle_key('x', T0)
        self.assertAlmostEqual(policy.scale, 1.0)

    def test_unknown_key_rejected_and_escape_bytes_ignored(self):
        policy = self.make_policy()
        for key in ('a', '\x1b', '[', 'A', 'y', '\n'):
            self.assertFalse(policy.handle_key(key, T0))
        self.assertEqual(policy.command(T0), (0.0, 0.0, 0.0))

    def test_invalid_limits_rejected(self):
        for bad in (0.0, -0.1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                M.TeleopPolicy(max_linear=bad, max_angular=1.0)
            with self.assertRaises(ValueError):
                M.TeleopPolicy(max_linear=0.1, max_angular=bad)
        with self.assertRaises(ValueError):
            M.TeleopPolicy(max_linear=0.1, max_angular=1.0, hold_timeout=0.0)


class BuildTwistTest(unittest.TestCase):

    def test_build_twist_maps_components(self):
        twist = M.build_twist((0.25, -0.5, 1.5))
        self.assertAlmostEqual(twist.linear.x, 0.25)
        self.assertAlmostEqual(twist.linear.y, -0.5)
        self.assertAlmostEqual(twist.angular.z, 1.5)


if __name__ == '__main__':
    unittest.main()
