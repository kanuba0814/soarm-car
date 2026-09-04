#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import sys
import unittest


MODULE_PATH = Path(__file__).parents[1] / 'soarm_navigation' / 'localization_cmd_gate.py'
SPEC = importlib.util.spec_from_file_location('localization_cmd_gate', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class GatePolicyTest(unittest.TestCase):
    def test_first_healthy_heartbeat_arms_and_forwards_fresh_command(self):
        policy = MODULE.GatePolicy(health_timeout=0.5, command_timeout=0.5)
        policy.update_health(MODULE.HEALTHY, 10.0)
        policy.update_command(10.1)
        self.assertEqual(policy.evaluate(10.2), (True, False))

    def test_health_timeout_stops_and_latches(self):
        policy = MODULE.GatePolicy(health_timeout=0.5, command_timeout=0.5)
        policy.update_health(MODULE.HEALTHY, 10.0)
        policy.update_command(10.1)
        self.assertEqual(policy.evaluate(10.2), (True, False))
        self.assertEqual(policy.evaluate(10.5), (False, True))
        policy.update_health(MODULE.HEALTHY, 10.6)
        policy.update_command(10.6)
        self.assertEqual(policy.evaluate(10.7), (False, False))

    def test_explicit_arm_requires_health_and_discards_old_command(self):
        policy = MODULE.GatePolicy(health_timeout=0.5, command_timeout=0.5)
        policy.update_health(MODULE.HEALTHY, 10.0)
        policy.update_command(10.1)
        policy.evaluate(10.2)
        policy.evaluate(10.5)
        self.assertFalse(policy.arm(10.5))
        policy.update_health(MODULE.HEALTHY, 10.6)
        self.assertFalse(policy.arm(10.7))
        policy.confirm_cancellation()
        self.assertTrue(policy.arm(10.7))
        self.assertEqual(policy.evaluate(10.7), (False, False))
        policy.update_command(10.71)
        self.assertEqual(policy.evaluate(10.72), (True, False))

    def test_command_timeout_is_fail_closed_without_latching_health(self):
        policy = MODULE.GatePolicy(health_timeout=0.5, command_timeout=0.2)
        policy.update_health(MODULE.HEALTHY, 10.0)
        policy.update_command(10.1)
        self.assertEqual(policy.evaluate(10.2), (True, False))
        self.assertEqual(policy.evaluate(10.3), (False, False))
        self.assertFalse(policy.latched)


if __name__ == '__main__':
    unittest.main()
