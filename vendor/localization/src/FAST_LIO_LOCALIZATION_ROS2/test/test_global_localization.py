#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np
import open3d as o3d
from builtin_interfaces.msg import Time
import tf_transformations


SCRIPTS_PATH = Path(__file__).parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS_PATH))
MODULE_PATH = SCRIPTS_PATH / 'global_localization.py'
SPEC = importlib.util.spec_from_file_location('global_localization', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
GlobalLocalizationNode = MODULE.GlobalLocalizationNode


class GlobalLocalizationRegistrationTest(unittest.TestCase):
    def test_tracking_skips_coarse_registration(self):
        calls = []

        def register(scan, submap, initial, scale):
            calls.append(scale)
            return initial, 0.9, 0.1

        node = SimpleNamespace(
            initialized=True,
            use_coarse_initialization=True,
            coarse_scale=5.0,
            registration_at_scale=register,
        )
        GlobalLocalizationNode.registration_candidate(
            node, object(), object(), np.eye(4))
        self.assertEqual(calls, [1.0])

    def test_initialization_can_use_coarse_registration(self):
        calls = []

        def register(scan, submap, initial, scale):
            calls.append(scale)
            return initial, 0.9, 0.1

        node = SimpleNamespace(
            initialized=False,
            use_coarse_initialization=True,
            coarse_scale=5.0,
            registration_at_scale=register,
        )
        GlobalLocalizationNode.registration_candidate(
            node, object(), object(), np.eye(4))
        self.assertEqual(calls, [5.0, 1.0])

    def test_tracking_rejects_rmse_and_correction_jumps(self):
        node = SimpleNamespace(
            initialized=True,
            localization_th=0.4,
            localization_rmse_th=0.3,
            max_correction_translation=0.25,
            max_correction_rotation=0.15,
            T_map_to_odom=np.eye(4),
        )
        accepted, reason = GlobalLocalizationNode.registration_is_acceptable(
            node, np.eye(4), 1.0, 0.31)
        self.assertFalse(accepted)
        self.assertIn('RMSE', reason)

        translated = np.eye(4)
        translated[0, 3] = 0.3
        accepted, reason = GlobalLocalizationNode.registration_is_acceptable(
            node, translated, 1.0, 0.1)
        self.assertFalse(accepted)
        self.assertIn('0.300 m', reason)

        rotated = np.eye(4)
        angle = 0.2
        rotated[:2, :2] = [[np.cos(angle), -np.sin(angle)],
                           [np.sin(angle), np.cos(angle)]]
        accepted, reason = GlobalLocalizationNode.registration_is_acceptable(
            node, rotated, 1.0, 0.1)
        self.assertFalse(accepted)
        self.assertIn('0.200 rad', reason)

        accepted, reason = GlobalLocalizationNode.registration_is_acceptable(
            node, np.eye(4), 0.8, 0.1)
        self.assertTrue(accepted, reason)

    def test_planar_gate_accepts_independent_vertical_correction(self):
        node = SimpleNamespace(
            initialized=True,
            localization_th=0.4,
            localization_rmse_th=0.3,
            max_correction_translation=0.25,
            max_correction_rotation=0.15,
            planar_correction_gate=True,
            max_correction_z=0.5,
            max_correction_roll_pitch=0.15,
            T_map_to_odom=np.eye(4),
        )
        candidate = np.eye(4)
        candidate[:3, 3] = [-0.034445, 0.0441, 0.389296]
        yaw = 0.033568
        candidate[:2, :2] = [[np.cos(yaw), -np.sin(yaw)],
                             [np.sin(yaw), np.cos(yaw)]]

        accepted, reason = GlobalLocalizationNode.registration_is_acceptable(
            node, candidate, 0.988, 0.140)
        self.assertTrue(accepted, reason)

        candidate[2, 3] = 0.5
        accepted, reason = GlobalLocalizationNode.registration_is_acceptable(
            node, candidate, 0.988, 0.140)
        self.assertFalse(accepted)
        self.assertIn('Z jump=0.500 m', reason)

        candidate = np.eye(4)
        candidate[0, 3] = 0.25
        accepted, reason = GlobalLocalizationNode.registration_is_acceptable(
            node, candidate, 0.988, 0.140)
        self.assertFalse(accepted)
        self.assertIn('XY jump=0.250 m', reason)

        candidate = tf_transformations.euler_matrix(0.15, 0.0, 0.0)
        accepted, reason = GlobalLocalizationNode.registration_is_acceptable(
            node, candidate, 0.988, 0.140)
        self.assertFalse(accepted)
        self.assertIn('roll/pitch jump=0.150 rad', reason)

        candidate = tf_transformations.euler_matrix(0.0, 0.0, 0.15)
        accepted, reason = GlobalLocalizationNode.registration_is_acceptable(
            node, candidate, 0.988, 0.140)
        self.assertFalse(accepted)
        self.assertIn('yaw jump=0.150 rad', reason)

    def test_acceptance_threshold_boundaries_are_exclusive(self):
        node = SimpleNamespace(
            initialized=True,
            localization_th=0.4,
            localization_rmse_th=0.3,
            max_correction_translation=0.25,
            max_correction_rotation=0.15,
            T_map_to_odom=np.eye(4),
        )
        accepted, _ = GlobalLocalizationNode.registration_is_acceptable(
            node, np.eye(4), 0.4, 0.1)
        self.assertFalse(accepted)
        accepted, _ = GlobalLocalizationNode.registration_is_acceptable(
            node, np.eye(4), 0.8, 0.3)
        self.assertFalse(accepted)

        candidate = np.eye(4)
        candidate[0, 3] = 0.25
        accepted, _ = GlobalLocalizationNode.registration_is_acceptable(
            node, candidate, 0.8, 0.1)
        self.assertFalse(accepted)

        angle = 0.150001
        candidate = np.eye(4)
        candidate[:2, :2] = [[np.cos(angle), -np.sin(angle)],
                             [np.sin(angle), np.cos(angle)]]
        accepted, _ = GlobalLocalizationNode.registration_is_acceptable(
            node, candidate, 0.8, 0.1)
        self.assertFalse(accepted)

        candidate[0, 3] = 0.249
        angle = 0.149
        candidate[:2, :2] = [[np.cos(angle), -np.sin(angle)],
                             [np.sin(angle), np.cos(angle)]]
        accepted, reason = GlobalLocalizationNode.registration_is_acceptable(
            node, candidate, 0.401, 0.299)
        self.assertTrue(accepted, reason)

    def test_non_finite_registration_results_are_rejected(self):
        node = SimpleNamespace(
            initialized=False,
            localization_th=0.4,
            localization_rmse_th=0.3,
        )
        for fitness, rmse in [(np.nan, 0.1), (np.inf, 0.1), (0.8, np.nan), (0.8, np.inf)]:
            accepted, reason = GlobalLocalizationNode.registration_is_acceptable(
                node, np.eye(4), fitness, rmse)
            self.assertFalse(accepted)
            self.assertIn('non-finite', reason)
        candidate = np.eye(4)
        candidate[0, 3] = np.nan
        accepted, reason = GlobalLocalizationNode.registration_is_acceptable(
            node, candidate, 0.8, 0.1)
        self.assertFalse(accepted)
        self.assertIn('non-finite', reason)

    def test_full_fov_uses_radial_distance_around_pose_estimate(self):
        global_map = o3d.geometry.PointCloud()
        global_map.points = o3d.utility.Vector3dVector(np.array([
            [10.5, 0.0, 0.0],
            [13.0, 0.0, 0.0],
            [5.0, 0.0, 0.0],
        ]))
        class Clock:
            class Now:
                @staticmethod
                def to_msg():
                    return Time()

            @staticmethod
            def now():
                return Clock.Now()

        class Publisher:
            @staticmethod
            def publish(message):
                pass

        node = SimpleNamespace(
            global_map=global_map,
            FOV=2 * np.pi,
            FOV_FAR=2.0,
            get_clock=lambda: Clock(),
            pub_submap=Publisher(),
        )
        pose_estimate = np.eye(4)
        pose_estimate[0, 3] = 10.0

        submap = GlobalLocalizationNode.crop_global_map_in_FOV(
            node, None, pose_estimate, object())
        points = np.asarray(submap.points)
        np.testing.assert_allclose(points, [[10.5, 0.0, 0.0]])


if __name__ == '__main__':
    unittest.main()
