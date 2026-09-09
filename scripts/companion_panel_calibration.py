#!/usr/bin/env python3
"""Evaluate fixed-color runtime-only calibration planes from saved RGB-D."""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image

from companion_reload_geometry import camera_transform, load_frame, to_world

PLANES = {
    "green_x": {"axis": 0, "value_m": -5.0, "mask": lambda rgb: (rgb[:, :, 1] > 150) & (rgb[:, :, 0] < 100) & (rgb[:, :, 2] < 100)},
    "blue_y": {"axis": 1, "value_m": 2.8, "mask": lambda rgb: (rgb[:, :, 2] > 150) & (rgb[:, :, 0] < 100) & (rgb[:, :, 1] < 100)},
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise SystemExit(f"Refusing to overwrite output directory: {args.output_dir}")
    capture = json.loads((args.input_dir / "capture.json").read_text())
    args.output_dir.mkdir(parents=True)
    measurements = []
    for record in capture["viewpoints"]:
        root = args.input_dir / "frames" / record["id"]
        depth, invalid, calibration = load_frame(args.input_dir, record)
        rgb = np.asarray(Image.open(root / "rgb.png").convert("RGB"))
        for name, plane in PLANES.items():
            mask = plane["mask"](rgb)
            valid = mask & (~invalid) & np.isfinite(depth) & (depth > 0)
            v, u = np.nonzero(valid)
            result = {"view_id": record["id"], "plane": name, "axis": "xyz"[plane["axis"]], "known_value_m": plane["value_m"], "fixed_color_pixels": int(mask.sum()), "valid_fixed_color_pixels": int(len(u))}
            if len(u) < 100:
                result.update({"status": "NOT_VERIFIED", "reason": "fewer than 100 fixed-color plane pixels"})
                measurements.append(result)
                continue
            fx, fy, cx, cy = calibration["k"][0], calibration["k"][4], calibration["k"][2], calibration["k"][5]
            z = depth[v, u].astype(np.float64)
            points = np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z))
            world = to_world(points, camera_transform(record))
            error = world[:, plane["axis"]] - plane["value_m"]
            result.update({"axis_error_median_m": float(np.median(error)), "axis_error_p95_abs_m": float(np.percentile(np.abs(error), 95)), "status": "DESCRIPTIVE_CALIBRATION_ONLY", "warning": "Fixed color and plane axis were declared before capture. This supports only the runtime calibration transform/depth convention, never furnished-home accuracy, target identity, or method evidence."})
            measurements.append(result)
    measured_axes = sorted({item["axis"] for item in measurements if item["status"] == "DESCRIPTIVE_CALIBRATION_ONLY"})
    report = {"scope": "runtime-only known green-X and blue-Y planes; never furnished-home targets, planner inputs, or evaluator ground truth", "fixed_masks": "green: G>150,R/B<100; blue: B>150,R/G<100", "measurements": measurements, "independent_axes_measured": measured_axes, "two_heading_axis_requirement_met": len(measured_axes) >= 2}
    (args.output_dir / "panel_calibration.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"measurements": len(measurements), "axes": measured_axes, "requirement": report["two_heading_axis_requirement_met"]}))


if __name__ == "__main__":
    main()
