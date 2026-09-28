#!/usr/bin/env python3
"""Adapt the posed B06 RGB-D sequence to Open3DIS's ScanNet reader contract.

The adapter deliberately consumes the measured RGB-D frames, validated odometry,
and OVIR-generated TSDF cloud.  It refuses invalid or incomplete trajectories and
never creates synthetic camera poses or 3D proposals.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import numpy as np
import open3d as o3d


def replace_symlink(link: Path, target: Path) -> None:
    if link.is_symlink():
        link.unlink()
    elif link.exists():
        raise FileExistsError(f"refusing to replace non-symlink: {link}")
    link.symlink_to(os.path.relpath(target.resolve(), link.parent.resolve()))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--odometry", type=Path, required=True)
    parser.add_argument("--tsdf-cloud", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene-id", default="B06")
    args = parser.parse_args()

    dataset = args.dataset.resolve()
    odometry_path = args.odometry.resolve()
    cloud_path = args.tsdf_cloud.resolve()
    manifest = json.loads((dataset / "manifest.json").read_text(encoding="utf-8"))
    odometry = json.loads(odometry_path.read_text(encoding="utf-8"))
    if not odometry.get("trajectory_valid"):
        raise ValueError("refusing Open3DIS adapter: odometry trajectory is invalid")

    frames = manifest["frames"]
    poses = odometry.get("poses", [])
    if len(frames) != len(poses):
        raise ValueError(f"frame/pose count mismatch: {len(frames)} != {len(poses)}")
    if not cloud_path.is_file():
        raise FileNotFoundError(f"missing TSDF cloud: {cloud_path}")

    output = args.output.resolve()
    color_dir = output / "color"
    depth_dir = output / "depth"
    pose_dir = output / "pose"
    for directory in (color_dir, depth_dir, pose_dir):
        directory.mkdir(parents=True, exist_ok=True)

    camera = manifest["camera_info"]["/camera/color/camera_info"]
    intrinsic = np.asarray(camera["k"], dtype=float).reshape(3, 3)
    for frame, pose in zip(frames, poses):
        index = int(frame["index"])
        if index != int(pose["index"]):
            raise ValueError(f"frame/pose index mismatch at {index}")
        frame_id = f"{index:06d}"
        # Open3DIS's ScanNetReader constructs .jpg names; OpenCV decodes the
        # linked PNG by content, avoiding lossy duplicate color data.
        replace_symlink(color_dir / f"{frame_id}.jpg", dataset / frame["rgb"])
        replace_symlink(depth_dir / f"{frame_id}.png", dataset / frame["depth"])
        pose_matrix = np.asarray(pose["world_from_camera"], dtype=float)
        if pose_matrix.shape != (4, 4) or not np.isfinite(pose_matrix).all():
            raise ValueError(f"invalid world_from_camera pose at frame {index}")
        np.savetxt(pose_dir / f"{frame_id}.txt", pose_matrix)

    np.savetxt(output / "intrinsic.txt", intrinsic)
    cloud = o3d.io.read_point_cloud(str(cloud_path))
    if cloud.is_empty():
        raise ValueError(f"TSDF cloud is empty: {cloud_path}")
    ply_path = output / f"{args.scene_id}.ply"
    if not o3d.io.write_point_cloud(str(ply_path), cloud):
        raise RuntimeError(f"failed to write Open3DIS point cloud: {ply_path}")

    metadata = {
        "schema": "soarm.open3dis_adapter.v1",
        "scene_id": args.scene_id,
        "frames": len(frames),
        "image_size": [int(camera["width"]), int(camera["height"])],
        "intrinsic": intrinsic.tolist(),
        "depth_scale": 1000.0,
        "pose_convention": odometry.get("pose_convention"),
        "pose_source": str(odometry_path),
        "tsdf_cloud_source": str(cloud_path),
        "point_cloud": str(ply_path),
        "point_count": len(cloud.points),
        "geometry_warning": "RGB-depth registration is not independently calibrated.",
        "three_d_proposals": "not fabricated; official ISBNet/spconv stage required",
    }
    (output / "adapter.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
