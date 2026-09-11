import hashlib
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from exploration_core import ExplorationCore, InputError, PriorMap, load_pinned_prior, ROBOT_RADIUS_M, NAV2_ARRIVAL_TOLERANCE_M

IDENTITY = "synthetic-map"
def prior(width=120, height=120, resolution=.05, occupied=()):
    pixels = bytearray([255] * (width * height))
    for x, y in occupied: pixels[(height - 1 - y) * width + x] = 0
    return PriorMap(width, height, resolution, (0., 0., 0.), bytes(pixels), IDENTITY)
def observation(identifier, revision, returns, sensor=(1., 1., 0.)):
    return {"id": identifier, "revision": revision, "map_identity": IDENTITY, "frame": "map", "source_stamp": "opaque", "sensor_origin": list(sensor), "robot_pose": [sensor[0], sensor[1], 0.], "returns": returns}
def dense_rays(kind="hit", z=.2, radius=1.6):
    # Synthetic finite rays, below max_rays; not a manually filled observed grid.
    return [{"xyz": [3. + radius * math.cos(2 * math.pi * i / 720),
                      3. + radius * math.sin(2 * math.pi * i / 720), z],
             "kind": kind} for i in range(720)]
def visible_square(core, low=10, high=110):
    core.visibility.update((x, y) for x in range(low, high) for y in range(low, high)); core.observed.update((x, y) for x in range(low, high) for y in range(low, high))

class CoreTests(unittest.TestCase):
    def test_a1_real_pinned_prior_is_unobserved(self):
        item = load_pinned_prior(ROOT / "config/prior_map_assisted_exploration_input_lock.json")
        self.assertEqual((item.width, item.height, item.resolution, item.origin), (320, 290, .05, (-8., -7.25, 0.)))
        self.assertEqual(item.map_identity, "house_floorplan-v1-7d9c0b34")
        self.assertEqual(ExplorationCore(item).observed, set())

    def test_measured_navigation_band_rays_produce_frontier_growth_and_duplicate(self):
        core = ExplorationCore(prior())
        record = observation("first", 1, dense_rays(), sensor=(3., 3., .2))
        self.assertLessEqual(len(record['returns']), core.max_rays)
        applied = core.apply_observation(record)
        self.assertGreater(applied["new_observed_cells"], 0)
        self.assertEqual(core.candidates([3., 3., 0.])["status"], "ok")
        self.assertEqual(core.apply_observation(record)["status"], "duplicate")
        grown = core.apply_observation(observation("second", 2, [{"xyz": [5.8, 1., 0.], "kind": "hit"}]))
        self.assertGreater(grown["new_observed_cells"], 0)
        with self.assertRaisesRegex(InputError, "stale"): core.apply_observation(observation("old", 2, []))
        wrong = observation("wrong", 3, []); wrong["map_identity"] = "other"
        with self.assertRaisesRegex(InputError, "identity"): core.apply_observation(wrong)
        with self.assertRaisesRegex(InputError, "nonfinite"): core.apply_observation(observation("nan", 3, [{"xyz": [float("nan"), 1, 0], "kind": "hit"}]))

    def test_ground_plane_height_is_translation_and_tilt_invariant(self):
        first = observation('plane-a', 1, [{'xyz': [2., 1., .2], 'kind': 'hit'}], sensor=(1., 1., .2))
        first['ground_plane_origin'], first['ground_plane_normal'] = [0., 0., 0.], [0., 0., 1.]
        shifted = observation('plane-b', 1, [{'xyz': [2., 1., 10.2], 'kind': 'hit'}], sensor=(1., 1., 10.2))
        shifted['ground_plane_origin'], shifted['ground_plane_normal'] = [0., 0., 10.], [0., 0., 1.]
        core_a, core_b = ExplorationCore(prior(), navigation_z_min=.03, navigation_z_max=1.5), ExplorationCore(prior(), navigation_z_min=.03, navigation_z_max=1.5)
        self.assertEqual(core_a.apply_observation(first)['new_observed_cells'], core_b.apply_observation(shifted)['new_observed_cells'])
        floor = observation('tilted-floor', 1, [{'xyz': [2., 1., .1], 'kind': 'hit'}], sensor=(1., 1., 0.))
        floor['ground_plane_origin'], floor['ground_plane_normal'] = [0., 0., 0.], [-.1, 0., math.sqrt(.99)]
        with self.assertRaisesRegex(InputError, 'unit'):
            ExplorationCore(prior()).apply_observation(dict(floor, ground_plane_normal=[0., 0., .9]))

    def test_floor_and_high_visibility_cannot_create_navigation_goal(self):
        for identifier, rays in (("floor", dense_rays("floor", 0.)), ("high", dense_rays("hit", 2.))):
            core = ExplorationCore(prior())
            core.apply_observation(observation(identifier, 1, rays, sensor=(3., 3., .2)))
            self.assertGreater(len(core.visibility), 1000)
            self.assertEqual(len(core.observed), 0)
            self.assertEqual(core.candidates([3., 3., 0.])["status"], "blocked")
        origin_high = ExplorationCore(prior())
        origin_high.apply_observation(observation("high-origin", 1, dense_rays(), sensor=(3., 3., 2.)))
        self.assertEqual(len(origin_high.observed), 0)

    def test_prior_and_live_obstacle_occlusion_is_history_safe(self):
        core = ExplorationCore(prior(80, 80, .05, occupied=[(30, y) for y in range(80)]))
        core.apply_observation(observation("wall", 1, [{"xyz": [2.5, 1., 0.], "kind": "hit"}]))
        self.assertNotIn((45, 20), core.visibility)
        core = ExplorationCore(prior())
        # The obstacle is collected before same-record rays, so return order cannot clear behind it.
        core.apply_observation(observation("same", 1, [{"xyz": [3., 1., 0.], "kind": "hit"}, {"xyz": [2., 1., 0.], "kind": "obstacle"}]))
        self.assertNotIn((60, 20), core.observed)
        core.apply_observation(observation("later", 2, [{"xyz": [3., 1., 0.], "kind": "hit"}]))
        self.assertNotIn((60, 20), core.observed)
        before = set(core.observed); core.apply_observation(observation("missing", 3, [{"kind": "no_return"}]))
        self.assertEqual(before, core.observed)

    def test_cached_configuration_space_matches_scalar_footprint_all_cells(self):
        core = ExplorationCore(prior(24, 21, .05, occupied=[(10, y) for y in range(4, 18)]))
        core.observed.update((x, y) for x in range(24) for y in range(21) if core.prior.occupancy((x, y)) == 'free')
        core.visibility.update(core.observed)
        core.live_obstacles.add((5, 5))
        cached = core._configuration_space()
        for x in range(core.prior.width):
            for y in range(core.prior.height):
                scalar = core._footprint_ok((x, y)) and (x, y) not in core.live_obstacles
                self.assertEqual(bool(cached[x, y]), scalar, (x, y))

    def test_footprint_corner_out_of_bounds_and_rotated_origin(self):
        core = ExplorationCore(prior()); visible_square(core); center = (35, 35)
        self.assertTrue(core._footprint_ok(center))
        corner = core.prior.world_to_cell((core.prior.cell_center(center)[0] + .275, core.prior.cell_center(center)[1] + .275))
        core.observed.remove(corner); self.assertFalse(core._footprint_ok(center)); self.assertFalse(core._footprint_ok((1, 1)))
        with self.assertRaisesRegex(InputError, "nonzero"): PriorMap(2, 2, .05, (0., 0., .1), bytes([255] * 4), IDENTITY)

    def test_retry_region_survives_frontier_change_and_reports_exhausted(self):
        core = ExplorationCore(prior()); visible_square(core)
        first = core.candidates([3., 3., 0.]); chosen = first["candidates"][0]
        core.report_result(chosen["cluster_id"], 0, False)
        self.assertNotIn((9, 20), core.observed)
        core.observed.add((9, 20)); core.visibility.add((9, 20))  # genuinely new adjacent observation
        second = core.candidates([3., 3., 0.]); second_id = second["candidates"][0]["cluster_id"]
        self.assertEqual(second_id, chosen["cluster_id"])
        core.report_result(second_id, 0, False)
        exhausted = core.candidates([3., 3., 0.])
        self.assertEqual(exhausted["status"], "exhausted")
        self.assertIn("retry_exhausted", [item["reason"] for item in exhausted["rejections"]])

    def test_arrival_cache_matches_scalar_and_rejects_recorded_unsafe_goal(self):
        core = ExplorationCore(prior(40, 40, occupied=[(3, y) for y in range(2, 5)]))
        core.observed.update((x, y) for x in range(40) for y in range(40) if core.prior.occupancy((x, y)) == 'free')
        core.visibility.update(core.observed)
        core.live_obstacles.add((30, 32))
        radius = ROBOT_RADIUS_M + NAV2_ARRIVAL_TOLERANCE_M
        cached = core._configuration_space(radius)
        self.assertTrue(cached.any())
        for x in range(40):
            for y in range(40):
                self.assertEqual(bool(cached[x, y]), core._clearance_ok(core.prior.cell_center((x, y)), radius))
        real = ExplorationCore(load_pinned_prior(ROOT / 'config/prior_map_assisted_exploration_input_lock.json'))
        real.observed.update((x, y) for x in range(real.prior.width) for y in range(real.prior.height) if real.prior.occupancy((x, y)) == 'free')
        # Geometric counterexamples reconstructed from the recorded map, not new motion.
        self.assertTrue(real._clearance_ok([-.825, 2.625], ROBOT_RADIUS_M))
        self.assertFalse(real._clearance_ok([-.825, 2.625], radius))
        self.assertFalse(real._clearance_ok([-.626, 2.625], ROBOT_RADIUS_M))
        real.live_obstacles.add((154, 193))
        pose = [-.765609578, 2.412325844, 0.]
        self.assertTrue(real._footprint_ok(real.prior.world_to_cell(pose)))
        self.assertFalse(real._clearance_ok(pose, ROBOT_RADIUS_M))

    def test_known_wall_and_map_edge_do_not_create_frontiers(self):
        map_with_wall = prior(3, 3, .05, occupied=[(x, y) for x in range(3) for y in range(3) if (x, y) != (1, 1)])
        core = ExplorationCore(map_with_wall)
        core.observed.add((1, 1))
        core.visibility.add((1, 1))
        result = core.candidates([.075, .075, 0.])
        self.assertEqual(result['status'], 'blocked')
        self.assertEqual(result['candidates'], [])

    def test_nonempty_frontier_without_safe_approach_is_blocked(self):
        core = ExplorationCore(prior()); core.observed.update({(20, 20), (21, 20)}); core.visibility.update(core.observed)
        result = core.candidates([1., 1., 0.])
        self.assertEqual(result["status"], "blocked")
        self.assertIn("start_clearance_occupied", [item["reason"] for item in result["rejections"]])
        # The old small coverage can contain the robot, but not a robust arrival goal.
        clear_start = ExplorationCore(prior())
        clear_start.apply_observation(observation("small", 1, dense_rays(radius=.8), sensor=(3., 3., .2)))
        result = clear_start.candidates([3., 3., 0.])
        self.assertEqual(result["status"], "blocked")
        self.assertIn("no_safe_approach", [item["reason"] for item in result["rejections"]])

    def test_checkpoint_preserves_frame_ids_and_retry_history(self):
        core = ExplorationCore(prior()); core.apply_observation(observation("one", 1, dense_rays(), sensor=(3., 3., .2)))
        selected = core.candidates([3., 3., 0.])["candidates"][0]; core.report_result(selected["cluster_id"], 0, False)
        state = core.export_state(); resumed = ExplorationCore(prior()); resumed.import_state(json.loads(json.dumps(state)))
        self.assertEqual(resumed.export_state(), state)
        self.assertEqual(resumed.apply_observation(observation("one", 2, []))["status"], "duplicate")
        invalid = dict(state); invalid["frame"] = "odom"
        with self.assertRaisesRegex(InputError, "incompatible"): ExplorationCore(prior()).import_state(invalid)

    def test_serialized_cli_synthetic_and_real_prior_consumer_seams(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); pgm = root / "map.pgm"; yaml_path = root / "map.yaml"
            pgm.write_bytes(b"P5\n120 120\n255\n" + bytes([255]) * 14400); yaml_path.write_text("image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n")
            manifest = root / "lock.json"; manifest.write_text(json.dumps({"map_identity": IDENTITY, "yaml_path": str(yaml_path), "pgm_path": str(pgm), "pgm_sha256": hashlib.sha256(pgm.read_bytes()).hexdigest(), "geometry": {"dimensions": [120, 120], "resolution": .05, "origin": [0., 0., 0.]}}))
            request, output = root / "request.json", root / "output.json"
            request.write_text(json.dumps({"prior_manifest": str(manifest), "robot_pose": [3., 3., 0.], "observations": [observation("wire", 1, dense_rays(), sensor=(3., 3., .2))]}))
            completed = subprocess.run([sys.executable, str(ROOT / "scripts/exploration_core.py"), str(request), str(output)], capture_output=True, text=True, check=False)
            self.assertEqual(completed.returncode, 0, completed.stderr); self.assertEqual(json.loads(output.read_text())["shortlist"]["status"], "ok")
            real = subprocess.run([sys.executable, str(ROOT / "scripts/exploration_core.py"), "examples/frontier-core-real-prior-request.json", str(root / "real.json")], cwd=ROOT, capture_output=True, text=True, check=False)
            self.assertEqual(real.returncode, 0, real.stderr); self.assertIn(json.loads((root / "real.json").read_text())["shortlist"]["status"], ("blocked", "exhausted", "no_frontier", "ok"))

if __name__ == "__main__": unittest.main()
