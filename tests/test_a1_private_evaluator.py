import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from evaluate_a1_visualconf_private import ground_truth_answers, max_weight_assignment, taxonomy

RULES = json.loads((Path(__file__).resolve().parents[1] / "config/a1_visualconf_dev_v1.json").read_text())["rules"]


class PrivateA1EvaluatorTests(unittest.TestCase):
    def instances(self, chair_point):
        return {
            "ChairA_01_001": {"identity": "ChairA_01_001", "label": "chair", "surface_voxels_2cm": [chair_point]},
            "KitchenTable_01_001": {"identity": "KitchenTable_01_001", "label": "dining table", "surface_voxels_2cm": [[0.0, 0.0, .4]]},
        }

    def test_explicit_taxonomy_does_not_confuse_tablet_or_coffee(self):
        self.assertEqual(taxonomy("KitchenTable_01_001"), "dining table")
        self.assertEqual(taxonomy("CoffeeTable_01_001"), "coffee table")
        self.assertEqual(taxonomy("Tablet_01_001"), "tablet")

    def test_assignment_is_one_to_one_not_greedy_duplicate_credit(self):
        predictions = [{"label": "chair", "box_xyxy": [0, 0, 10, 10]}, {"label": "chair", "box_xyxy": [0, 0, 10, 10]}]
        targets = [{"label": "chair", "bbox_xyxy": [0, 0, 10, 10], "identity": "ChairA_01_001", "label_id": 9}]
        matches, _ = max_weight_assignment(predictions, targets)
        self.assertEqual(len(matches), 1)
        self.assertEqual(matches[0][0]["identity"], "ChairA_01_001")

    def test_right_positive_axis_classifies_h0_y_positive_as_left(self):
        self.assertEqual(ground_truth_answers(self.instances([0.0, 1.0, .4]), 0.0, RULES)["Q3"]["identities"], ["ChairA_01_001"])

    def test_right_positive_axis_classifies_h90_x_negative_as_left(self):
        self.assertEqual(ground_truth_answers(self.instances([-1.0, 0.0, .4]), math.pi / 2, RULES)["Q3"]["identities"], ["ChairA_01_001"])


if __name__ == "__main__":
    unittest.main()
