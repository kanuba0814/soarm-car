import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from a1_visualconf.public import associate, camera_transform, make_records, score_tracks


class PublicA1InvariantTests(unittest.TestCase):
    def detection(self, observation_id, index, box, pixels, xy):
        return {"observation_id": observation_id, "detection_index": index, "label": "chair", "confidence": .8, "box_xyxy": box, "foreground": {"failure": None, "pixels_rc": pixels, "world_points": [[xy[0], xy[1], .4], [xy[0] + .02, xy[1], .4]], "xy_median_m": xy, "sensitivity": {"max_xy_shift_m": .01}}}

    def test_twenty_same_view_duplicates_do_not_increase_m(self):
        detections = [self.detection("obs_001", index, [0, 0, 20, 20], [[0, 0], [0, 1]], [1.0, 1.0]) for index in range(20)]
        track = {"track_id": "track_001", "label": "chair", "detections": detections, "status": "active"}
        score_tracks([track])
        self.assertEqual(track["M"], 0.0)
        self.assertEqual(track["M_evidence"]["distinct_observations"], 1)

    def test_missing_anchor_is_relation_unknown(self):
        chair = {"track_id": "track_001", "label": "chair", "detections": [self.detection("obs_001", 0, [0, 0, 20, 20], [[0, 0], [0, 1]], [1.0, 1.0])], "status": "active"}
        score_tracks([chair])
        config = {"queries": [{"id": "Q3", "kind": "leftmost_chair_left_table"}]}
        rows = make_records(config, [chair], 0.0)
        relation_row = next(row for row in rows if row["variant"] == "V+R")
        self.assertIsNone(relation_row["accepted_track_id"])
        self.assertEqual(relation_row["relation_status"], "unknown")
        self.assertEqual(relation_row["abstain_reason"], "missing_or_ambiguous_anchor")

    def test_same_image_distinct_chairs_are_not_merged(self):
        first = self.detection("obs_001", 0, [0, 0, 20, 20], [[0, 0], [0, 1]], [1.0, 1.0])
        second = self.detection("obs_001", 1, [40, 0, 60, 20], [[0, 40], [0, 41]], [1.05, 1.0])
        tracks, audit = associate([first, second], {})
        self.assertEqual(len(tracks), 2)
        self.assertEqual(audit["same_frame_duplicates"], [])

    def test_rotated_mount_transform_uses_full_quaternion(self):
        pose = {"entity": {"position": {"x": 1.0, "y": 2.0, "z": 3.0}, "orientation": {"x": 0.0, "y": 0.0, "z": math.sqrt(.5), "w": math.sqrt(.5)}}}
        mounting = {"base_to_camera_translation_m": [.18, 0.0, .2263], "optical_to_base_rows": [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]}
        origin, rotation = camera_transform(pose, mounting)
        np.testing.assert_allclose(origin, [1.0, 2.18, 3.2263], atol=1e-8)
        np.testing.assert_allclose(rotation @ np.array([0.0, 0.0, 1.0]), [0.0, 1.0, 0.0], atol=1e-8)


if __name__ == "__main__":
    unittest.main()
