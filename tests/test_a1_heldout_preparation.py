import hashlib
import json
import subprocess
import tempfile
import unittest
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from run_a1_heldout_testbatch import commands, load_layouts, renderer_environment


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

    def test_command_construction_uses_unique_locked_worlds(self):
        batch = json.loads(Path("artifacts/companion-home-validation/evidence/a1-heldout-testbatch-v2/batch-plan.json").read_text())
        layouts = load_layouts(batch, Path("artifacts/companion-home-validation/evidence/a1-heldout-layouts-v1"))
        class Args:
            survey_config = "config/companion_instance_survey.json"
            a1_config = "config/a1_visualconf_dev_v1.json"
            yolo_python = "/home/gxxl/.cache/soarm-perception-envs/conceptgraphs-b06/bin/python3.10"
        constructed = [commands(layout, Path("/tmp/a1-runtime"), Args()) for layout in layouts]
        self.assertEqual(len({str(layout["world"]) for layout in layouts}), 3)
        self.assertEqual(len({layout["world_sha256"] for layout in layouts}), 3)
        environments = [renderer_environment(layout) for layout in layouts]
        self.assertEqual(len({item["A1_WORLD_OVERRIDE"] for item in environments}), 3)
        self.assertEqual(len({item["A1_WORLD_OVERRIDE_SHA256"] for item in environments}), 3)
        self.assertTrue(all(chain[0][0].endswith("python") or "python" in chain[0][0] for chain in constructed))
        self.assertTrue(all("companion_capture.py" in chain[0][1] and "evaluate_a1_visualconf_private.py" in chain[-1][1] for chain in constructed))

    def test_batch_plan_refuses_existing_root(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "existing"
            output.mkdir()
            result = subprocess.run(["python3", "scripts/plan_a1_heldout_testbatch.py", "--layouts", "config/a1_heldout_layouts_v1.json", "--a1-config", "config/a1_visualconf_dev_v1.json", "--output-root", str(output)], capture_output=True)
            self.assertNotEqual(result.returncode, 0)


if __name__ == "__main__":
    unittest.main()
