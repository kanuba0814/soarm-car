import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from run_vlm_grid_format_v2 import exact_simple_id


class FormatV2Tests(unittest.TestCase):
    def test_accepts_only_exact_simple_answer(self):
        self.assertEqual(exact_simple_id(" 1 ", {"0", "1"}), "1")
        self.assertEqual(exact_simple_id(" NONE ", {"0", "1"}), "NONE")

    def test_does_not_recover_punctuation_or_letter(self):
        self.assertIsNone(exact_simple_id("1.", {"0", "1"}))
        self.assertIsNone(exact_simple_id("A.", {"0", "1"}))


if __name__ == "__main__":
    unittest.main()
