import copy
import json
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from a1_visualconf.public import associate, camera_transform, connected_components_with_rules, make_records, score_tracks

RULES = json.loads((Path(__file__).resolve().parents[1] / "config/a1_visualconf_dev_v1.json").read_text())["rules"]


class PublicA1InvariantTests(unittest.TestCase):
    def detection(self, observation_id, index, box, pixels, xy, label="chair"):
        return {"observation_id": observation_id, "detection_index": index, "label": label, "confidence": .8, "box_xyxy": box, "foreground": {"failure": None, "pixels_rc": pixels, "world_points": [[xy[0], xy[1], .4], [xy[0] + .02, xy[1], .4]], "xy_median_m": xy, "sensitivity": {"max_xy_shift_m": .01, "measurements": []}}}

    def relation_tracks(self, chair_y=.5):
        chair = {"track_id": "chair", "label": "chair", "detections": [self.detection("obs_001", 0, [0, 0, 20, 20], [[0, 0], [0, 1]], [1.0, chair_y])], "status": "active"}
        table_detection = self.detection("obs_001", 1, [30, 0, 60, 20], [[0, 30], [0, 31]], [1.0, 0.0], "dining table")
        table = {"track_id": "table", "label": "dining table", "detections": [table_detection], "status": "active"}
        score_tracks([chair, table], RULES)
        return chair, table

    def relation_row(self, tracks, kind="leftmost_chair_left_table"):
        rows = make_records({"rules": RULES, "queries": [{"id": "Q3", "kind": kind}]}, tracks, 0.0)
        return next(row for row in rows if row["variant"] == "V+R")

    def test_twenty_same_view_duplicates_do_not_increase_m(self):
        detections = [self.detection("obs_001", index, [0, 0, 20, 20], [[0, 0], [0, 1]], [1.0, 1.0]) for index in range(20)]
        track = {"track_id": "track_001", "label": "chair", "detections": detections, "status": "active"}
        score_tracks([track], RULES)
        self.assertEqual(track["M"], 0.0)
        self.assertEqual(track["M_evidence"]["distinct_observations"], 1)

    def test_missing_anchor_is_relation_unknown(self):
        chair = {"track_id": "track_001", "label": "chair", "detections": [self.detection("obs_001", 0, [0, 0, 20, 20], [[0, 0], [0, 1]], [1.0, 1.0])], "status": "active"}
        score_tracks([chair], RULES)
        row = self.relation_row([chair])
        self.assertIsNone(row["accepted_track_id"])
        self.assertEqual(row["relation_status"], "unknown")
        self.assertEqual(row["abstain_reason"], "missing_or_ambiguous_anchor")

    def test_same_image_distinct_chairs_are_not_merged(self):
        first = self.detection("obs_001", 0, [0, 0, 20, 20], [[0, 0], [0, 1]], [1.0, 1.0])
        second = self.detection("obs_001", 1, [40, 0, 60, 20], [[0, 40], [0, 41]], [1.05, 1.0])
        tracks, audit = associate([first, second], {}, RULES)
        self.assertEqual(len(tracks), 2)
        self.assertEqual(audit["same_frame_duplicates"], [])

    def test_rotated_mount_transform_uses_full_quaternion(self):
        pose = {"entity": {"position": {"x": 1.0, "y": 2.0, "z": 3.0}, "orientation": {"x": 0.0, "y": 0.0, "z": math.sqrt(.5), "w": math.sqrt(.5)}}}
        mounting = {"base_to_camera_translation_m": [.18, 0.0, .2263], "optical_to_base_rows": [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]}
        origin, rotation = camera_transform(pose, mounting)
        np.testing.assert_allclose(origin, [1.0, 2.18, 3.2263], atol=1e-8)
        np.testing.assert_allclose(rotation @ np.array([0.0, 0.0, 1.0]), [0.0, 1.0, 0.0], atol=1e-8)

    def test_right_positive_axis_classifies_h0_y_positive_as_left(self):
        chair, table = self.relation_tracks(.5)
        row = self.relation_row([chair, table])
        self.assertEqual(row["accepted_track_id"], "chair")

    def test_right_positive_axis_classifies_h90_x_negative_as_left(self):
        chair, table = self.relation_tracks(.5)
        chair["relation_surface_envelopes"] = [[[-1.0, 0.0, .4], [-.98, 0.0, .4]]]
        table["relation_surface_envelopes"] = [[[0.0, 0.0, .4], [.02, 0.0, .4]]]
        rows = make_records({"rules": RULES, "queries": [{"id": "Q3", "kind": "leftmost_chair_left_table"}]}, [chair, table], math.pi / 2)
        self.assertEqual(next(row for row in rows if row["variant"] == "V+R")["accepted_track_id"], "chair")

    def test_all_chairs_on_wrong_side_abstain(self):
        chair, table = self.relation_tracks(-.5)
        row = self.relation_row([chair, table])
        self.assertIsNone(row["accepted_track_id"])
        self.assertEqual(row["abstain_reason"], "no_definite_left_side")

    def test_anchor_perturbation_reversal_abstains(self):
        chair, table = self.relation_tracks(.4)
        table["relation_surface_envelopes"].append([[1.0, .6, .4], [1.02, .6, .4]])
        row = self.relation_row([chair, table])
        self.assertIsNone(row["accepted_track_id"])
        self.assertEqual(row["relation_status"], "unknown")

    def test_failed_perturbation_is_unknown(self):
        chair, table = self.relation_tracks(.5)
        table["relation_geometry_failures"] = [{"measurement": {"failure": "foreground_ambiguous"}}]
        row = self.relation_row([chair, table])
        self.assertEqual(row["abstain_reason"], "perturbation_geometry_failed")

    def test_threshold_change_changes_component_gate(self):
        valid = np.array([[True, True]])
        depth = np.array([[1.0, 1.06]])
        strict = copy.deepcopy(RULES)
        permissive = copy.deepcopy(RULES)
        strict["component_depth_abs_m"] = .05
        permissive["component_depth_abs_m"] = .07
        self.assertEqual(len(connected_components_with_rules(valid, depth, strict)), 2)
        self.assertEqual(len(connected_components_with_rules(valid, depth, permissive)), 1)


if __name__ == "__main__":
    unittest.main()
