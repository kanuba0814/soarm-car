#!/usr/bin/env python3
"""Prepare a posed B06 RGB-D sequence for OpenFusion's ScanNet dataset adapter.

The output deliberately follows OpenFusion's native ``ScanNet`` layout.  Pose
files contain ``world_from_camera`` (the ScanNet adapter inverts them before
TSDF integration); no pose is estimated or synthesized here.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any


def read_contract(dataset: Path, odometry: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    manifest = json.loads((dataset / "manifest.json").read_text(encoding="utf-8"))
    trajectory = json.loads(odometry.read_text(encoding="utf-8"))
    if not trajectory.get("trajectory_valid"):
        raise ValueError("refusing invalid odometry trajectory")
    frames, poses = manifest.get("frames", []), trajectory.get("poses", [])
    if not frames or len(frames) != len(poses):
        raise ValueError(f"frame/pose count mismatch: {len(frames)} != {len(poses)}")
    for frame, pose in zip(frames, poses):
        if int(frame["index"]) != int(pose["index"]):
            raise ValueError(f"frame/pose index mismatch: {frame['index']} != {pose['index']}")
        matrix = pose.get("world_from_camera")
        if not isinstance(matrix, list) or len(matrix) != 4 or any(len(row) != 4 for row in matrix):
            raise ValueError(f"invalid world_from_camera matrix for frame {frame['index']}")
    return manifest, trajectory


def replace_link(link: Path, source: Path) -> None:
    if link.exists() or link.is_symlink():
        if not link.is_symlink():
            raise FileExistsError(f"refusing to replace non-symlink: {link}")
        link.unlink()
    link.symlink_to(os.path.relpath(source.resolve(), link.parent.resolve()))


def consumer_check(upstream: Path, output: Path) -> None:
    """Instantiate the exact pinned OpenFusion ScanNet consumer and read sample 0."""
    code = """\
import json
from openfusion.datasets import ScanNet
output = __import__('sys').argv[1]
dataset = ScanNet(output, -1, False)
assert len(dataset.rgbs_list) == len(dataset.depths_list) == len(dataset.pose_list)
rgb, depth, extrinsic = next(iter(dataset))
print(json.dumps({'frames': len(dataset), 'rgb': rgb, 'depth': depth,
                  'extrinsic_shape': list(extrinsic.shape)}))
"""
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(upstream) + os.pathsep + environment.get("PYTHONPATH", "")
    subprocess.run([sys.executable, "-c", code, str(output)], check=True, env=environment)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--odometry", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--consumer-check", action="store_true")
    parser.add_argument("--upstream", type=Path, help="pinned OpenFusion checkout for --consumer-check")
    args = parser.parse_args()

    dataset, odometry, output = args.dataset.resolve(), args.odometry.resolve(), args.output.resolve()
    manifest, trajectory = read_contract(dataset, odometry)
    for directory in (output / "rgb", output / "depth", output / "pose", output / "intrinsic"):
        directory.mkdir(parents=True, exist_ok=True)

    for frame, pose in zip(manifest["frames"], trajectory["poses"]):
        frame_id = f"{int(frame['index']):06d}"
        replace_link(output / "rgb" / f"{frame_id}.png", dataset / frame["rgb"])
        replace_link(output / "depth" / f"{frame_id}.png", dataset / frame["depth"])
        (output / "pose" / f"{frame_id}.txt").write_text(
            "\n".join(" ".join(f"{float(value):.17g}" for value in row) for row in pose["world_from_camera"]) + "\n",
            encoding="utf-8",
        )

    camera = manifest["camera_info"]["/camera/color/camera_info"]
    intrinsic = [camera["k"][0:3], camera["k"][3:6], camera["k"][6:9]]
    (output / "intrinsic" / "intrinsic_depth.txt").write_text(
        "\n".join(" ".join(f"{float(value):.17g}" for value in row) for row in intrinsic) + "\n", encoding="utf-8"
    )
    metadata = {
        "schema": "soarm.openfusion.scannet_adapter.v1",
        "source_manifest": str((dataset / "manifest.json")),
        "source_odometry": str(args.odometry.resolve()),
        "frames": len(manifest["frames"]),
        "pose_convention": "world_from_camera; OpenFusion ScanNet consumer inverts this to its TSDF extrinsic",
        "depth_scale": 1000.0,
        "rgb_depth_registration": "not independently verified; no registration transform was fabricated",
        "license_note": "OpenFusion repository root does not provide a top-level license; assess license before redistribution.",
    }
    (output / "adapter.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    if args.consumer_check:
        if args.upstream is None:
            parser.error("--consumer-check requires --upstream")
        consumer_check(args.upstream.resolve(), output)
    print(json.dumps({"output": str(output), "frames": metadata["frames"], "consumer_checked": args.consumer_check}, indent=2))


if __name__ == "__main__":
    main()
