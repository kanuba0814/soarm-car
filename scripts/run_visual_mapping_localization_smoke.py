#!/usr/bin/env python3
"""Run separate pure-RGB-D mapping and localization bundle smoke checks."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile

import numpy as np

from soarm_visual_localization import CameraProfile, GuardedRGBDOdometry, RGBDFrame, VisualMap, VisualState, depth_to_points


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=12)
    parser.add_argument("--seed", type=int, default=23)
    args = parser.parse_args()
    width, height = 64, 48
    profile = CameraProfile(width, height, 60.0, 60.0, 31.5, 23.5,
                            depth_scale=0.001, depth_min_m=0.2,
                            depth_max_m=4.0, registration_verified=True)
    rng = np.random.default_rng(args.seed)
    base = rng.integers(700, 1800, size=(height, width), dtype=np.uint16)
    tracker = GuardedRGBDOdometry(profile, min_inliers=200, min_observability=0.001)
    visual_map = VisualMap(profile)
    states = []
    for index in range(args.frames):
        depth = np.clip(base.astype(np.int32) + index, 300, 3000).astype(np.uint16)
        frame = RGBDFrame(np.zeros((height, width, 3), dtype=np.uint8), depth,
                          index * 0.05 + 1.0, "camera_link")
        result = tracker.update(frame)
        states.append(result.state.value)
        if result.accepted and (index == 0 or index % 3 == 0):
            points, _ = depth_to_points(depth, profile)
            visual_map.add(frame.stamp, result.transform, points)
    with tempfile.TemporaryDirectory() as directory:
        manifest = visual_map.save_bundle(directory, map_id="visual-smoke-v1")
        loaded, map_id = VisualMap.load_bundle(directory)
        query = depth_to_points(base, profile)[0]
        confirmations = []
        keyframe, pose, rmse = None, np.eye(4), float("inf")
        for _ in range(3):
            keyframe, pose, rmse = loaded.relocalize(query)
            confirmations.append(bool(keyframe is not None and np.isfinite(rmse)))
        bundle_digest = (Path(directory) / "bundle.sha256").read_text(encoding="utf-8").strip()
        del manifest, pose
    output = {
        "schema": "soarm.visual_mapping_localization_smoke.v1",
        "radar_inputs": [],
        "truth_inputs": [],
        "frames": args.frames,
        "seed": args.seed,
        "mapping_states": states,
        "mapping_tracking_frames": states.count(VisualState.TRACKING.value),
        "keyframes": len(loaded.keyframes),
        "map_id": map_id,
        "relocalization_confirmed_candidate": keyframe is not None,
        "relocalization_confirmation_frames": sum(confirmations),
        "relocalization_rmse_m": float(rmse),
        "bundle_sha256": bundle_digest,
        "profile": profile.__dict__,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
