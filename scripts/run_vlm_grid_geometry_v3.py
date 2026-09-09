#!/usr/bin/env python3
"""Replay corrected CPU-only configuration-space geometry on saved public DEV RGB-D."""
import argparse
import json
from pathlib import Path

import numpy as np

from a1_visualconf.public import camera_transform
from run_vlm_grid_demo import candidate_targets, render_grid, world_points
from vlm_grid.geometry import FREE, OCCUPIED, UNKNOWN, build_grid, configuration_space_grid, reachable_approach


def coverage(grid):
    return {"free_cells": int((grid == FREE).sum()), "occupied_cells": int((grid == OCCUPIED).sum()), "unknown_cells": int((grid == UNKNOWN).sum())}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--public-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--v1-results", type=Path, required=True)
    parser.add_argument("--v2-results", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise SystemExit("refusing to overwrite output directory")
    config = json.loads(args.config.read_text())
    calibration = json.loads((args.public_dir / "calibration_selfpose.json").read_text())["observations"]
    detections = json.loads((args.public_dir / "detections.json").read_text())["observations"]
    v1 = {record["observation_id"]: record for record in json.loads(args.v1_results.read_text())["records"]}
    v2 = json.loads(args.v2_results.read_text())
    args.output_dir.mkdir(parents=True)
    overlays = args.output_dir / "overlays"
    overlays.mkdir()
    records = []
    for observation_id in config["observations"]:
        root = args.public_dir / observation_id
        depth = np.load(root / "depth_m_float32.npy")
        info = json.loads((root / "camera_info.json").read_text())
        transform = camera_transform(calibration[observation_id]["self_pose"], calibration[observation_id]["camera_mounting"])
        source_xy = transform[0][:2]
        candidates = candidate_targets(depth, info, transform, [dict(item, observation_id=observation_id) for item in detections[observation_id]])
        points = world_points(depth, info, transform, config["grid"]["ray_stride_px"], config["grid"]["max_depth_m"])
        obstacles = np.abs(points[:, 2]) > config["grid"]["floor_clearance_m"]
        raw_grid, origin = build_grid(source_xy, points, obstacles, config["grid"]["resolution_m"], config["grid"]["robot_radius_m"])
        configured = configuration_space_grid(raw_grid, round(config["grid"]["robot_radius_m"] / config["grid"]["resolution_m"]))
        candidate_results = {}
        for candidate in candidates:
            if candidate["target_xy_m"] is None:
                candidate_results[candidate["candidate_id"]] = {"status": "rejected", "reason": candidate["target_status"]}
                continue
            route, reason = reachable_approach(configured, origin, config["grid"]["resolution_m"], source_xy, candidate["target_xy_m"], config["grid"]["approach_radius_m"])
            candidate_results[candidate["candidate_id"]] = {"status": "feasible", "grid": route} if route else {"status": "rejected", "reason": reason}
        feasible = [candidate for candidate in candidates if candidate_results[candidate["candidate_id"]]["status"] == "feasible"]
        choice = min(feasible, key=lambda item: candidate_results[item["candidate_id"]]["grid"]["route_length_m"])["candidate_id"] if feasible else "NO_FEASIBLE_CANDIDATE"
        render_grid(overlays / f"{observation_id}_configuration_grid.png", configured, origin, config["grid"]["resolution_m"], source_xy, candidates, candidate_results)
        records.append({"observation_id": observation_id, "raw_observation_grid_coverage": coverage(raw_grid), "configuration_space_grid_coverage": coverage(configured), "candidates": candidates, "candidate_results": candidate_results, "geometry_only_choice": choice, "v1_geometry_only_choice": v1[observation_id]["decisions"]["geometry_only"]["choice"]})
    result = {"scope": "DEV_ONLY_CPU_CONFIGURATION_SPACE_REPLAY", "algorithm": "BFS shortest 4-connected path on configuration-space FREE cells, not A*", "footprint_rule": "A robot center is FREE only if every in-bounds disk cell within the 0.50 m radius is observed FREE; UNKNOWN, OCCUPIED, and out-of-bounds disk cells block it.", "raycast_limit": "Free cells are conditional single-view RGB-D height-classification evidence, not a collision proof.", "source_artifacts": {"public_input": str(args.public_dir), "v1_geometry_results": str(args.v1_results), "v2_format_results": str(args.v2_results), "v2_observations": v2["summary"]["observations"]}, "records": records, "summary": {"observations": len(records), "v1_geometry_feasible": sum(record["v1_geometry_only_choice"] != "NO_FEASIBLE_CANDIDATE" for record in records), "v3_geometry_feasible": sum(record["geometry_only_choice"] != "NO_FEASIBLE_CANDIDATE" for record in records)}, "replay": "python scripts/run_vlm_grid_geometry_v3.py --public-dir artifacts/companion-home-validation/evidence/vlm-grid-dev-v1/public-input --config config/vlm_grid_dev_v1.json --v1-results artifacts/companion-home-validation/evidence/vlm-grid-dev-v1/results/results.json --v2-results artifacts/companion-home-validation/evidence/vlm-grid-dev-v2/results/results.json --output-dir <new-output-dir>"}
    (args.output_dir / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result["summary"]))


if __name__ == "__main__":
    main()
