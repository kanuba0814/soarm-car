#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace


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

    def test_expected_map_and_operator_stop_require_fresh_health_and_drain(self):
        policy = MODULE.GatePolicy(health_timeout=0.5, command_timeout=0.5, expected_map_id='bundle-compatible-id')
        policy.update_health(MODULE.HEALTHY, 10.0, 'wrong-id')
        policy.update_command(10.1)
        self.assertEqual(policy.evaluate(10.2), (False, False))
        policy.update_health(MODULE.HEALTHY, 10.3, 'bundle-compatible-id')
        policy.update_command(10.31)
        self.assertEqual(policy.evaluate(10.32), (True, False))
        policy.force_stop()
        self.assertEqual(policy.evaluate(10.33), (False, False))
        self.assertFalse(policy.arm(10.34))
        policy.confirm_cancellation()
        self.assertTrue(policy.arm(10.34))

    def test_command_timeout_is_fail_closed_without_latching_health(self):
        policy = MODULE.GatePolicy(health_timeout=0.5, command_timeout=0.2)
        policy.update_health(MODULE.HEALTHY, 10.0)
        policy.update_command(10.1)
        self.assertEqual(policy.evaluate(10.2), (True, False))
        self.assertEqual(policy.evaluate(10.3), (False, False))
        self.assertFalse(policy.latched)


class CancellationDrainTest(unittest.TestCase):
    class Future:
        def __init__(self, result): self.value = result
        def done(self): return True
        def result(self): return self.value

    class Client:
        def __init__(self, result): self.value = result
        def service_is_ready(self): return True
        def call_async(self, request): return CancellationDrainTest.Future(self.value)

    @staticmethod
    def status(goal_id=None, state=None):
        statuses = [] if goal_id is None else [SimpleNamespace(status=state, goal_info=SimpleNamespace(goal_id=SimpleNamespace(uuid=[goal_id] * 16)))]
        return SimpleNamespace(status_list=statuses)

    def gate(self, goals_canceling=()):
        gate = object.__new__(MODULE.LocalizationCommandGate)
        response = SimpleNamespace(return_code=0, goals_canceling=[SimpleNamespace(goal_id=SimpleNamespace(uuid=[goal_id] * 16)) for goal_id in goals_canceling])
        gate.policy = MODULE.GatePolicy(.5, .5); gate.policy.force_stop()
        gate.cancel_clients = [self.Client(response), self.Client(response)]
        gate.cancel_futures = [None, None]; gate.cancel_confirmed = [False, False]
        gate.nav_nonterminal = [False, False]; gate.drain_required = [False, False]
        gate.nav_active_ids = [set(), set()]; gate.terminal_since_stop = [set(), set()]
        gate.status_seen_after_cancel = [False, False]; gate.status_generation = [0, 0]
        gate.cancel_status_generation = [0, 0]; gate.cancel_goal_ids = [set(), set()]
        gate.cancel_epoch = 0; gate.last_cancel_sweep = 0
        gate.begin_cancellation(1.0)
        return gate

    def test_empty_status_before_cancel_ack_cannot_drain_known_goal(self):
        gate = self.gate((7,))
        gate.advance_cancellation()
        gate.save_navigation_status(self.status(), 0)
        gate.save_navigation_status(self.status(), 1)
        gate.advance_cancellation()
        self.assertFalse(gate.policy.cancellation_complete)
        gate.save_navigation_status(self.status(7, MODULE.GoalStatus.STATUS_CANCELED), 0)
        gate.save_navigation_status(self.status(7, MODULE.GoalStatus.STATUS_CANCELED), 1)
        gate.advance_cancellation()
        self.assertTrue(gate.policy.cancellation_complete)

    def test_unused_servers_fresh_empty_reply_and_late_goal(self):
        gate = self.gate()
        gate.advance_cancellation()
        self.assertFalse(gate.policy.cancellation_complete)
        gate.advance_cancellation()
        self.assertTrue(gate.policy.cancellation_complete)
        gate.save_navigation_status(self.status(9, MODULE.GoalStatus.STATUS_ACCEPTED), 0)
        self.assertFalse(gate.policy.cancellation_complete)
        gate.advance_cancellation()
        gate.advance_cancellation()
        self.assertFalse(gate.policy.cancellation_complete)
        gate.save_navigation_status(self.status(9, MODULE.GoalStatus.STATUS_CANCELED), 0)
        gate.advance_cancellation()
        self.assertTrue(gate.policy.cancellation_complete)

    def test_cancel_ack_goal_requires_canceling_then_terminal_status(self):
        gate = self.gate((7,)); gate.advance_cancellation()
        self.assertFalse(gate.policy.cancellation_complete)
        gate.save_navigation_status(self.status(7, MODULE.GoalStatus.STATUS_CANCELING), 0)
        gate.save_navigation_status(self.status(7, MODULE.GoalStatus.STATUS_CANCELING), 1); gate.advance_cancellation()
        self.assertFalse(gate.policy.cancellation_complete)
        gate.save_navigation_status(self.status(7, MODULE.GoalStatus.STATUS_CANCELED), 0)
        gate.save_navigation_status(self.status(7, MODULE.GoalStatus.STATUS_CANCELED), 1)
        gate.advance_cancellation()
        self.assertTrue(gate.policy.cancellation_complete)


if __name__ == '__main__':
    unittest.main()
