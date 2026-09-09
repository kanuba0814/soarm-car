import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from exploration_core import ExplorationCore, InputError, PriorMap, ROBOT_RADIUS_M, load_pinned_prior

IDENTITY = "synthetic-map"
def prior(width=80, height=80, resolution=.05, occupied=()):
    pixels = bytearray([255] * (width * height))
    for x, y in occupied:
        pixels[(height - 1 - y) * width + x] = 0
    return PriorMap(width, height, resolution, (0., 0., 0.), bytes(pixels), IDENTITY)
def observation(identifier, revision, returns, sensor=(1., 1., 0.)):
    return {"id": identifier, "revision": revision, "map_identity": IDENTITY, "frame": "map", "source_stamp": "opaque", "sensor_origin": list(sensor), "robot_pose": [1., 1., 0.], "returns": returns}
def visible_square(core, low=15, high=55):
    core.observed.update((x, y) for x in range(low, high) for y in range(low, high))

class CoreTests(unittest.TestCase):
    def test_a1_real_pinned_prior_is_unobserved(self):
        item = load_pinned_prior(ROOT / "config/prior_map_assisted_exploration_input_lock.json")
        self.assertEqual((item.width, item.height, item.resolution, item.origin), (320, 290, .05, (-8., -7.25, 0.)))
        self.assertEqual(item.map_identity, "house_floorplan-v1-7d9c0b34")
        self.assertEqual(ExplorationCore(item).observed, set())

    def test_measured_growth_duplicate_and_rejections(self):
        core = ExplorationCore(prior())
        # Finite measured rays cover a known square; this is sensor evidence, not copied prior free space.
        rays = [{"xyz": [x * .05 + .025, y * .05 + .025, 0.], "kind": "hit"} for x in range(15, 55) for y in range(15, 55)]
        first = observation("first", 1, rays)
        applied = core.apply_observation(first)
        self.assertGreater(applied["new_observed_cells"], 0)
        self.assertEqual(core.candidates([1.75, 1.75, 0.])["status"], "ok")
        self.assertEqual(core.apply_observation(first)["status"], "duplicate")
        grown = core.apply_observation(observation("second", 2, [{"xyz": [2.8, 1., 0.], "kind": "hit"}]))
        self.assertGreater(grown["new_observed_cells"], 0)
        with self.assertRaisesRegex(InputError, "stale"): core.apply_observation(observation("old", 2, []))
        wrong = observation("wrong", 3, []); wrong["map_identity"] = "other"
        with self.assertRaisesRegex(InputError, "identity"): core.apply_observation(wrong)
        with self.assertRaisesRegex(InputError, "nonfinite"): core.apply_observation(observation("nan", 3, [{"xyz": [float("nan"), 1, 0], "kind": "hit"}]))

    def test_wall_occlusion_floor_and_missing_do_not_create_free_space(self):
        core = ExplorationCore(prior(80, 80, .05, occupied=[(30, y) for y in range(80)]))
        core.apply_observation(observation("wall", 1, [{"xyz": [2.5, 1., 0.], "kind": "hit"}]))
        self.assertNotIn((45, 20), core.observed)  # no observation through the prior wall at x=30
        before = set(core.observed)
        core.apply_observation(observation("missing", 2, [{"kind": "no_return"}]))
        self.assertEqual(before, core.observed)
        core.apply_observation(observation("floor", 3, [{"xyz": [1.5, 1., 5.], "kind": "floor"}]))
        self.assertNotIn((40, 20), core.observed)

    def test_footprint_corner_unknown_out_of_bounds_and_rotated_origin(self):
        core = ExplorationCore(prior())
        visible_square(core)
        center = (35, 35)
        self.assertTrue(core._footprint_ok(center))
        # This cell intersects the .474m enclosing circle although old .35m logic misses it.
        corner = core.prior.world_to_cell((core.prior.cell_center(center)[0] + .275, core.prior.cell_center(center)[1] + .275))
        core.observed.remove(corner)
        self.assertFalse(core._footprint_ok(center))
        self.assertFalse(core._footprint_ok((1, 1)))
        with self.assertRaisesRegex(InputError, "nonzero"):
            PriorMap(2, 2, .05, (0., 0., .1), bytes([255] * 4), IDENTITY)

    def test_frontier_connectivity_and_bounded_failure(self):
        core = ExplorationCore(prior())
        visible_square(core)
        # An observed island is not connected to the current robot component.
        core.observed.update((x, y) for x in range(65, 70) for y in range(65, 70))
        result = core.candidates([1.75, 1.75, 0.])
        self.assertEqual(result["status"], "ok")
        self.assertTrue(result["candidates"])
        chosen = result["candidates"][0]
        self.assertLess(chosen["goal"][0], 3.)
        core.report_result(chosen["cluster_id"], 0, False)
        core.report_result(chosen["cluster_id"], 0, False)
        self.assertNotIn(chosen["cluster_id"], [item["cluster_id"] for item in core.candidates([1.75, 1.75, 0.])["candidates"]])

    def test_serialized_cli_consumer_seam(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); pgm = root / "map.pgm"; yaml_path = root / "map.yaml"
            pgm.write_bytes(b"P5\n40 40\n255\n" + bytes([255]) * 1600)
            yaml_path.write_text("image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n")
            manifest = root / "lock.json"; manifest.write_text(json.dumps({"map_identity": IDENTITY, "yaml_path": str(yaml_path), "pgm_path": str(pgm), "pgm_sha256": hashlib.sha256(pgm.read_bytes()).hexdigest(), "geometry": {"dimensions": [40, 40], "resolution": .05, "origin": [0., 0., 0.]}}))
            request = root / "request.json"; output = root / "output.json"
            request.write_text(json.dumps({"prior_manifest": str(manifest), "robot_pose": [1., 1., 0.], "observations": [observation("wire", 1, [{"xyz": [1.5, 1., 0.], "kind": "hit"}])]}))
            completed = subprocess.run([sys.executable, str(ROOT / "scripts/exploration_core.py"), str(request), str(output)], capture_output=True, text=True, check=False)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            produced = json.loads(output.read_text())
            self.assertEqual(produced["observation_results"][0]["status"], "applied")
            self.assertIn(produced["shortlist"]["status"], ("ok", "blocked", "no_frontier", "exhausted"))

if __name__ == "__main__": unittest.main()
