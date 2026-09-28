#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("color_frames", type=Path)
    parser.add_argument("depth_frames", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--fps", type=float, default=1.0)
    parser.add_argument("--min-depth-mm", type=int, default=300)
    parser.add_argument("--max-depth-mm", type=int, default=3000)
    args = parser.parse_args()

    color_paths = sorted(args.color_frames.glob("*.png"))
    depth_paths = sorted(args.depth_frames.glob("*.png"))
    if not color_paths or len(color_paths) != len(depth_paths):
        raise RuntimeError("Color/depth frame counts must be equal and non-zero")

    first = cv2.imread(str(color_paths[0]))
    height, width = first.shape[:2]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(args.output), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (width * 2, height)
    )
    if not writer.isOpened():
        raise RuntimeError(f"Could not open {args.output}")

    results = []
    for index, (color_path, depth_path) in enumerate(zip(color_paths, depth_paths), start=1):
        color = cv2.imread(str(color_path))
        depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        valid = (depth >= args.min_depth_mm) & (depth <= args.max_depth_mm)
        invalid_ratio = 1.0 - float(np.count_nonzero(valid)) / valid.size
        median_mm = float(np.median(depth[valid])) if np.any(valid) else 0.0

        normalized = np.clip(
            (depth.astype(np.float32) - args.min_depth_mm)
            / (args.max_depth_mm - args.min_depth_mm),
            0,
            1,
        )
        depth_color = cv2.applyColorMap((255 * (1 - normalized)).astype(np.uint8), cv2.COLORMAP_TURBO)
        depth_color[~valid] = 0
        combined = np.hstack([color, depth_color])
        cv2.putText(combined, "RGB", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(
            combined,
            "DEPTH (unregistered)",
            (width + 12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            combined,
            f"valid={100 * (1-invalid_ratio):.1f}% median={median_mm:.0f} mm",
            (width + 12, height - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        writer.write(combined)
        results.append({"frame": index, "valid_ratio": 1-invalid_ratio, "median_depth_mm": median_mm})

    writer.release()
    args.output.with_suffix(".json").write_text(
        json.dumps({"depth_unit": "mm", "registered_to_color": False, "frames": results}, indent=2),
        encoding="utf-8",
    )
    print(args.output)


if __name__ == "__main__":
    main()
