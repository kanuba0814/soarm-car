import hashlib
import json
import subprocess
import tempfile
import unittest
from pathlib import Path


class HeldoutPreparationTests(unittest.TestCase):
    def test_generation_changes_geometry_deterministically_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source.sdf"
            source.write_text("<sdf><world name='default'><model name='ChairA_01_001'><pose>1 0 0 0 0 0</pose></model><model name='ChairA_01_002'><pose>0 1 0 0 0 0</pose></model><model name='ChairA_01_003'><pose>-1 0 0 0 0 0</pose></model></world></sdf>")
            plan = {"source_asset_lock": {"world_sha256": hashlib.sha256(source.read_bytes()).hexdigest()}, "geometry_checks": {"chair_radius_m": .1, "minimum_changed_pair_clearance_m": .01, "world_xy_bounds_m": [-3, 3, -3, 3], "limitation": "synthetic"}, "layouts": [{"scene_id": "synthetic", "seed": 101, "transforms": [{"name": "ChairA_01_001", "radial_shift_m": .4, "tangential_shift_m": .1}, {"name": "ChairA_01_002", "radial_shift_m": .4, "tangential_shift_m": -.1}, {"name": "ChairA_01_003", "radial_shift_m": .4, "tangential_shift_m": .1}]}]}
            layout = root / "plan.json"
            layout.write_text(json.dumps(plan))
            output = root / "output"
            command = ["python3", "scripts/prepare_a1_heldout_layout.py", "--source-world", str(source), "--layouts", str(layout), "--scene-id", "synthetic", "--output-dir", str(output)]
            subprocess.run(command, check=True)
            lock = json.loads((output / "layout-lock.json").read_text())
            self.assertEqual(len(lock["changed_transforms"]), 3)
            self.assertNotEqual(lock["changed_transforms"][0]["before_pose"], lock["changed_transforms"][0]["after_pose"])
            self.assertNotEqual(lock["source_world_sha256"], lock["generated_world_sha256"])
            self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)

    def test_batch_plan_refuses_existing_root(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "existing"
            output.mkdir()
            result = subprocess.run(["python3", "scripts/plan_a1_heldout_testbatch.py", "--layouts", "config/a1_heldout_layouts_v1.json", "--a1-config", "config/a1_visualconf_dev_v1.json", "--output-root", str(output)], capture_output=True)
            self.assertNotEqual(result.returncode, 0)


if __name__ == "__main__":
    unittest.main()
