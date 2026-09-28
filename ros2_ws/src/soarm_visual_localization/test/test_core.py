import json
import tempfile
import unittest

import numpy as np

from soarm_visual_localization import (
    CameraProfile,
    GuardedRGBDOdometry,
    RGBDFrame,
    VisualMap,
    VisualState,
    depth_to_points,
)


class RGBDCoreTest(unittest.TestCase):
    def setUp(self):
        self.profile = CameraProfile(
            width=32, height=24, fx=30.0, fy=30.0, cx=15.5, cy=11.5,
            depth_scale=0.001, depth_min_m=0.2, depth_max_m=4.0,
            registration_verified=True,
        )

    def frame(self, stamp, depth):
        return RGBDFrame(np.zeros((24, 32, 3), dtype=np.uint8), depth, stamp, "camera_link")

    def test_depth_backprojection_uses_millimetre_scale(self):
        depth = np.full((24, 32), 1000, dtype=np.uint16)
        points, pixels = depth_to_points(depth, self.profile)
        self.assertEqual(len(points), 768)
        self.assertAlmostEqual(float(np.median(points[:, 2])), 1.0)
        self.assertEqual(pixels.shape, (768, 2))

    def test_unverified_registration_is_fail_closed(self):
        profile = CameraProfile(**{**self.profile.__dict__, "registration_verified": False})
        tracker = GuardedRGBDOdometry(profile)
        result = tracker.update(self.frame(1.0, np.full((24, 32), 1000, dtype=np.uint16)))
        self.assertEqual(result.state, VisualState.DEGRADED)
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "rgb_depth_registration_unverified")

    def test_small_registered_translation_is_accepted(self):
        rng = np.random.default_rng(4)
        depth = rng.integers(700, 1600, size=(24, 32), dtype=np.uint16)
        tracker = GuardedRGBDOdometry(self.profile, min_inliers=100, min_observability=0.001)
        self.assertEqual(tracker.update(self.frame(1.0, depth)).state, VisualState.TRACKING)
        moved = np.clip(depth.astype(np.int32) + 2, 300, 3000).astype(np.uint16)
        result = tracker.update(self.frame(1.05, moved))
        self.assertEqual(result.state, VisualState.TRACKING)
        self.assertGreater(result.inliers, 100)
        self.assertTrue(np.isfinite(result.rmse_m))

    def test_stale_timestamp_rejected(self):
        depth = np.full((24, 32), 1000, dtype=np.uint16)
        tracker = GuardedRGBDOdometry(self.profile)
        tracker.update(self.frame(2.0, depth))
        result = tracker.update(self.frame(1.9, depth))
        self.assertEqual(result.state, VisualState.DEGRADED)
        self.assertEqual(result.reason, "stale_or_nonmonotonic_timestamp")

    def test_robust_update_rejects_depth_outlier_in_residual(self):
        rng = np.random.default_rng(12)
        depth = rng.integers(700, 1600, size=(24, 32), dtype=np.uint16)
        tracker = GuardedRGBDOdometry(self.profile, min_inliers=100, trim_fraction=0.8,
                                      min_observability=0.001)
        tracker.update(self.frame(1.0, depth))
        moved = np.clip(depth.astype(np.int32) + 2, 300, 3000).astype(np.uint16)
        moved[5:10, 5:10] = 3500
        result = tracker.update(self.frame(1.05, moved))
        self.assertEqual(result.state, VisualState.TRACKING)
        self.assertLess(result.rmse_m, 0.01)

    def test_visual_map_bundle_round_trip_and_relocalization(self):
        points = np.array([[0.0, 0.0, 1.0], [0.2, 0.0, 1.2], [0.0, 0.2, 1.1],
                           [-0.1, 0.1, 0.9]])
        visual_map = VisualMap(self.profile)
        visual_map.add(1.0, np.eye(4), points)
        with tempfile.TemporaryDirectory() as directory:
            visual_map.save_bundle(directory, map_id="visual-bundle-v1")
            loaded, map_id = VisualMap.load_bundle(directory)
            keyframe, pose, rmse = loaded.relocalize(points)
        self.assertEqual(map_id, "visual-bundle-v1")
        self.assertIsNotNone(keyframe)
        self.assertTrue(np.allclose(pose, np.eye(4), atol=1e-6))
        self.assertAlmostEqual(rmse, 0.0, delta=1e-6)

    def test_invalid_depth_pixels_are_not_cross_paired(self):
        rng = np.random.default_rng(9)
        depth = rng.integers(700, 1600, size=(24, 32), dtype=np.uint16)
        moved = depth.copy()
        depth[0, 0] = 0
        moved[0, 1] = 0
        tracker = GuardedRGBDOdometry(self.profile, min_inliers=100)
        tracker.update(self.frame(1.0, depth))
        result = tracker.update(self.frame(1.05, moved))
        self.assertEqual(result.state, VisualState.TRACKING)
        self.assertLess(result.rmse_m, 1e-6)

    def test_visual_map_save_has_profile_and_keyframes(self):
        points = np.array([[0.0, 0.0, 1.0], [0.2, 0.0, 1.2], [0.0, 0.2, 1.1]])
        visual_map = VisualMap(self.profile)
        visual_map.add(1.0, np.eye(4), points)
        with tempfile.TemporaryDirectory() as directory:
            path = f"{directory}/map.json"
            visual_map.save(path, map_id="visual-test-v1")
            with open(path, encoding="utf-8") as stream:
                document = json.load(stream)
        self.assertEqual(document["schema"], "soarm.visual_map.v1")
        self.assertEqual(document["map_id"], "visual-test-v1")
        self.assertEqual(len(document["keyframes"]), 1)
    def test_input_adapter_normalizes_and_records_stream_stamps(self):
        from soarm_visual_localization import RGBDInputAdapter
        adapter = RGBDInputAdapter(self.profile, max_sync_skew_s=0.02)
        color = np.zeros((24, 32, 3), dtype=np.uint8)
        depth = np.full((24, 32), 1000, dtype=np.uint16)
        frame = adapter.make_frame(color, depth, 1.0, "camera_link",
                                   color_stamp=1.000, depth_stamp=1.010,
                                   camera_info_stamp=1.005)
        self.assertEqual(frame.frame_id, "camera_link")
        self.assertEqual(frame.depth_stamp, 1.010)

    def test_input_adapter_rejects_timestamp_skew(self):
        from soarm_visual_localization import RGBDInputAdapter
        adapter = RGBDInputAdapter(self.profile, max_sync_skew_s=0.02)
        with self.assertRaisesRegex(ValueError, "timestamp skew"):
            adapter.make_frame(np.zeros((24, 32, 3), dtype=np.uint8),
                               np.full((24, 32), 1000, dtype=np.uint16), 1.0,
                               "camera_link", color_stamp=1.0, depth_stamp=1.1)

    def test_input_adapter_rejects_sparse_depth(self):
        from soarm_visual_localization import RGBDInputAdapter
        adapter = RGBDInputAdapter(self.profile, min_valid_depth_ratio=0.2)
        depth = np.zeros((24, 32), dtype=np.uint16)
        with self.assertRaisesRegex(ValueError, "valid depth ratio"):
            adapter.make_frame(np.zeros((24, 32, 3), dtype=np.uint8), depth, 1.0,
                               "camera_link")

    def test_patch_rgbd_frontend_tracks_textured_pair(self):
        from soarm_visual_localization import PatchRGBDOdometry
        rng = np.random.default_rng(31)
        color = (rng.random((24, 32, 3)) * 255).astype(np.uint8)
        depth = np.full((24, 32), 1000, dtype=np.uint16)
        tracker = PatchRGBDOdometry(self.profile, min_features=5,
                                     min_observability=0.001)
        first = tracker.update(RGBDFrame(color, depth, 1.0, "camera_link"))
        second = tracker.update(RGBDFrame(np.roll(color, 1, axis=1), depth,
                                          1.05, "camera_link"))
        self.assertEqual(first.state, VisualState.TRACKING)
        self.assertIn(second.state, (VisualState.TRACKING, VisualState.DEGRADED))

    def test_ransac_rigid_rejects_correspondence_outliers(self):
        from soarm_visual_localization.frontend import _ransac_rigid
        rng = np.random.default_rng(22)
        source = rng.normal(size=(30, 3))
        target = source + np.array([0.02, -0.01, 0.03])
        target[-6:] = rng.normal(size=(6, 3))
        rotation, translation, mask = _ransac_rigid(source, target, threshold_m=0.01)
        self.assertGreaterEqual(int(mask.sum()), 23)
        self.assertTrue(np.allclose(rotation, np.eye(3), atol=1e-6))
        self.assertTrue(np.allclose(translation, [0.02, -0.01, 0.03], atol=1e-6))


if __name__ == "__main__":
    unittest.main()
