#!/usr/bin/env python3
"""Convert the standard B06 RGB-D export into FARM's frames.json contract."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path


def replace_symlink(link: Path, target: Path) -> None:
    if link.is_symlink():
        link.unlink()
    elif link.exists():
        raise FileExistsError(f"refusing to replace non-symlink: {link}")
    link.symlink_to(os.path.relpath(target.resolve(), link.parent.resolve()), target_is_directory=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--odometry", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    dataset = args.dataset.resolve()
    manifest = json.loads((dataset / "manifest.json").read_text(encoding="utf-8"))
    odometry = json.loads(args.odometry.read_text(encoding="utf-8"))
    if not odometry.get("trajectory_valid"):
        raise ValueError("odometry trajectory is not valid")

    source_frames = manifest["frames"]
    poses = odometry["poses"]
    if len(source_frames) != len(poses):
        raise ValueError(f"frame/pose count mismatch: {len(source_frames)} != {len(poses)}")

    camera_info = manifest["camera_info"]["/camera/color/camera_info"]
    K_flat = camera_info["k"]
    K = [K_flat[0:3], K_flat[3:6], K_flat[6:9]]

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    replace_symlink(output / "rgb", dataset / "rgb")
    replace_symlink(output / "depth", dataset / "depth")

    frames = []
    for source, pose in zip(source_frames, poses):
        index = int(source["index"])
        if index != int(pose["index"]):
            raise ValueError(f"frame/pose index mismatch at {index}")
        frames.append(
            {
                "camera": "astra_s",
                "frame_id": f"{index:06d}",
                "rgb_path": source["rgb"],
                "depth_path": source["depth"],
                "depth_size": [int(camera_info["height"]), int(camera_info["width"])],
                "K": K,
                "T_world_cam": pose["world_from_camera"],
                "timestamp_ns": int(source["color_header_ns"]),
                "sync_delta_ms": float(source["sync_delta_ms"]),
                "sync_ok": bool(source["sync_ok"]),
            }
        )

    farm_index = {
        "schema": "farm.frames_json.v1",
        "cameras": ["astra_s"],
        "depth_encoding": {
            "format": "png16",
            "scale_to_metres": 0.001,
            "invalid_value": 0,
        },
        "source_manifest": str((dataset / "manifest.json").resolve()),
        "source_odometry": str(args.odometry.resolve()),
        "geometry_warning": "Relative RGB-D odometry only; RGB-depth registration is not independently verified.",
        "frames": frames,
    }
    destination = output / "frames.json"
    destination.write_text(json.dumps(farm_index, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {destination} ({len(frames)} frames)")


if __name__ == "__main__":
    main()
