#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).parents[1] / 'scripts' / 'localization_health.py'
SPEC = importlib.util.spec_from_file_location('localization_health', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class LocalizationHealthTest(unittest.TestCase):
    def test_consecutive_rejections_fail_closed(self):
        tracker = MODULE.LocalizationHealthTracker(max_consecutive_rejections=3)
        self.assertEqual(tracker.state, MODULE.INITIALIZING)

        self.assertTrue(tracker.accepted(100, fitness=0.9, rmse=0.1))
        self.assertEqual(tracker.state, MODULE.HEALTHY)
        self.assertEqual(tracker.last_accepted_stamp_ns, 100)

        tracker.rejected(MODULE.REASON_QUALITY)
        tracker.rejected(MODULE.REASON_QUALITY)
        self.assertEqual(tracker.state, MODULE.HEALTHY)
        tracker.rejected(MODULE.REASON_QUALITY)
        self.assertEqual(tracker.state, MODULE.DEGRADED)
        self.assertEqual(tracker.consecutive_rejections, 3)

    def test_degraded_state_requires_explicit_reinitialization(self):
        tracker = MODULE.LocalizationHealthTracker(max_consecutive_rejections=1)
        tracker.accepted(100, fitness=0.9, rmse=0.1)
        tracker.rejected(MODULE.REASON_AMBIGUOUS)
        self.assertEqual(tracker.state, MODULE.DEGRADED)

        self.assertFalse(tracker.accepted(200, fitness=0.95, rmse=0.05))
        self.assertEqual(tracker.state, MODULE.DEGRADED)
        tracker.reinitialize()
        self.assertEqual(tracker.state, MODULE.INITIALIZING)
        self.assertTrue(tracker.accepted(300, fitness=0.95, rmse=0.05))
        self.assertEqual(tracker.state, MODULE.HEALTHY)

    def test_map_output_requires_fresh_healthy_heartbeat(self):
        self.assertTrue(MODULE.map_output_allowed(False, None, None, 10.0, 0.5))
        self.assertTrue(MODULE.map_output_allowed(True, MODULE.HEALTHY, 9.51, 10.0, 0.5))
        self.assertFalse(MODULE.map_output_allowed(True, MODULE.HEALTHY, 9.50, 10.0, 0.5))
        self.assertFalse(MODULE.map_output_allowed(True, MODULE.DEGRADED, 9.99, 10.0, 0.5))
        self.assertFalse(MODULE.map_output_allowed(True, None, None, 10.0, 0.5))
        self.assertTrue(MODULE.map_output_allowed(
            True, MODULE.HEALTHY, 9.99, 10.0, 0.5, 'house.pcd', 'house.pcd'))
        self.assertFalse(MODULE.map_output_allowed(
            True, MODULE.HEALTHY, 9.99, 10.0, 0.5, 'house.pcd', 'other.pcd'))

    def test_invalid_registration_metrics_fail_closed(self):
        tracker = MODULE.LocalizationHealthTracker(max_consecutive_rejections=1)
        self.assertFalse(tracker.accepted(1, float('nan'), 0.1))
        self.assertEqual(tracker.reason_mask, MODULE.REASON_INTERNAL)
        tracker.reinitialize()
        self.assertFalse(tracker.accepted(1, 0.9, float('inf')))
        self.assertEqual(tracker.reason_mask, MODULE.REASON_INTERNAL)

    def test_invalid_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            MODULE.LocalizationHealthTracker(max_consecutive_rejections=0)
        with self.assertRaises(ValueError):
            MODULE.map_output_allowed(True, MODULE.HEALTHY, 1.0, 2.0, 0.0)


if __name__ == '__main__':
    unittest.main()
