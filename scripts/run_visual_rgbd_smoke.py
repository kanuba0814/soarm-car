#!/usr/bin/env python3
"""Run a deterministic no-radar RGB-D geometry smoke benchmark."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np

from soarm_visual_localization import CameraProfile, GuardedRGBDOdometry, RGBDFrame, VisualState


def sequence(width: int, height: int, count: int, seed: int):
    rng = np.random.default_rng(seed)
    base = rng.integers(700, 1800, size=(height, width), dtype=np.uint16)
    for index in range(count):
        yield base.astype(np.uint16), index * 0.05


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--seed", type=int, default=17)
    args = parser.parse_args()
    width, height = 64, 48
    profile = CameraProfile(width, height, 60.0, 60.0, 31.5, 23.5,
                            depth_scale=0.001, depth_min_m=0.2,
                            depth_max_m=4.0, registration_verified=True)
    tracker = GuardedRGBDOdometry(profile, min_inliers=200, min_observability=0.001)
    states = []
    elapsed = []
    for depth, stamp in sequence(width, height, args.frames, args.seed):
        frame = RGBDFrame(np.zeros((height, width, 3), dtype=np.uint8), depth, stamp + 1.0, "camera_link")
        started = time.perf_counter()
        result = tracker.update(frame)
        elapsed.append((time.perf_counter() - started) * 1000.0)
        states.append(result.state.value)
    output = {
        "schema": "soarm.visual_rgbd_smoke.v1",
        "radar_inputs": [],
        "truth_inputs": [],
        "frames": args.frames,
        "seed": args.seed,
        "states": states,
        "tracking_frames": states.count(VisualState.TRACKING.value),
        "mean_ms": float(np.mean(elapsed)),
        "p95_ms": float(np.quantile(elapsed, 0.95)),
        "profile": profile.__dict__,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
