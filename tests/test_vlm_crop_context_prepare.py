import unittest

from PIL import Image

from scripts.vlm_crop_context.prepare import clip_box_xyxy, prepare_views


class PrepareViewsTest(unittest.TestCase):
    def test_clips_and_preserves_original_box_and_hash(self):
        image = Image.new("RGB", (20, 10), "red")
        detection = {"observation_id": "obs_x", "detection_index": 7, "box_xyxy": [-3.0, 2.0, 12.0, 12.0]}
        prepared = prepare_views(image, [detection], ["5"])
        candidate = prepared["candidates"][0]
        self.assertEqual(candidate["original_box_xyxy"], detection["box_xyxy"])
        self.assertEqual(candidate["clipped_box_xyxy"], [0.0, 2.0, 12.0, 10.0])
        self.assertEqual(candidate["source_rgb_sha256"], prepared["source_rgb_sha256"])
        self.assertEqual(prepared["full_annotated"].mode, "RGB")
        self.assertGreater(candidate["crop_source_size_px"][0], 0)

    def test_rejects_nonfinite_and_empty_boxes(self):
        with self.assertRaises(ValueError):
            clip_box_xyxy([0, 0, float("nan"), 4], (10, 10))
        with self.assertRaises(ValueError):
            clip_box_xyxy([20, 0, 25, 4], (10, 10))


if __name__ == "__main__":
    unittest.main()
