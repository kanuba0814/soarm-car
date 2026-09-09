import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from evaluate_a1_visualconf_private import max_weight_assignment, taxonomy


class PrivateA1EvaluatorTests(unittest.TestCase):
    def test_explicit_taxonomy_does_not_confuse_tablet_or_coffee(self):
        self.assertEqual(taxonomy("KitchenTable_01_001"), "dining table")
        self.assertEqual(taxonomy("CoffeeTable_01_001"), "coffee table")
        self.assertEqual(taxonomy("Tablet_01_001"), "tablet")

    def test_assignment_is_one_to_one_not_greedy_duplicate_credit(self):
        predictions = [
            {"label": "chair", "box_xyxy": [0, 0, 10, 10]},
            {"label": "chair", "box_xyxy": [0, 0, 10, 10]},
        ]
        targets = [
            {"label": "chair", "bbox_xyxy": [0, 0, 10, 10], "identity": "ChairA_01_001", "label_id": 9},
        ]
        matches, _ = max_weight_assignment(predictions, targets)
        self.assertEqual(len(matches), 1)
        self.assertEqual(matches[0][0]["identity"], "ChairA_01_001")


if __name__ == "__main__":
    unittest.main()
