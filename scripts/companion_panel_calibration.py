#!/usr/bin/env python3
"""Evaluate a runtime-only known green panel using saved RGB-D and fixed transform assumptions."""
import argparse
import json
from pathlib import Path

import numpy as np

from companion_reload_geometry import camera_transform, load_frame, to_world


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--plane-world-x", type=float, default=-5.0)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise SystemExit(f"Refusing to overwrite output directory: {args.output_dir}")
    capture = json.loads((args.input_dir / "capture.json").read_text())
    args.output_dir.mkdir(parents=True)
    views = []
    for record in capture["viewpoints"]:
        root = args.input_dir / "frames" / record["id"]
        depth, invalid, calibration = load_frame(args.input_dir, record)
        from PIL import Image
        rgb = np.asarray(Image.open(root / "rgb.png").convert("RGB"))
        panel = (rgb[:, :, 1] > 150) & (rgb[:, :, 0] < 100) & (rgb[:, :, 2] < 100)
        valid = panel & (~invalid) & np.isfinite(depth) & (depth > 0)
        v, u = np.nonzero(valid)
        if len(u) < 100:
            views.append({"id": record["id"], "green_pixels": int(panel.sum()), "valid_panel_pixels": int(len(u)), "status": "NOT_VERIFIED", "reason": "fewer than 100 independently color-selected panel pixels"})
            continue
        fx, fy, cx, cy = calibration["k"][0], calibration["k"][4], calibration["k"][2], calibration["k"][5]
        z = depth[v, u].astype(np.float64)
        points = np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z))
        world = to_world(points, camera_transform(record))
        x_error = world[:, 0] - args.plane_world_x
        views.append({"id": record["id"], "green_pixels": int(panel.sum()), "valid_panel_pixels": int(len(u)), "known_plane_world_x_m": args.plane_world_x, "world_x_error_median_m": float(np.median(x_error)), "world_x_error_p95_abs_m": float(np.percentile(np.abs(x_error), 95)), "status": "DESCRIPTIVE_CALIBRATION_ONLY", "warning": "Fixed green-color selection precedes world-plane comparison; this checks the declared transform/depth convention but is not home-study accuracy or method evidence."})
    report = {"scope": "runtime-only calibration panel; never a furnished-home target, planner input, or evaluator ground truth", "views": views}
    (args.output_dir / "panel_calibration.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"views": len(views), "measured": sum("world_x_error_median_m" in item for item in views)}))


if __name__ == "__main__":
    main()
