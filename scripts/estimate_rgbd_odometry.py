#!/usr/bin/env python3
"""Estimate a relative camera trajectory from an exported RGB-D sequence."""

import argparse
import json
from pathlib import Path

import numpy as np
import open3d as o3d


def load_rgbd(root, frame, depth_scale, depth_trunc):
    color = o3d.io.read_image(str(root / frame["rgb"]))
    depth = o3d.io.read_image(str(root / frame["depth"]))
    return o3d.geometry.RGBDImage.create_from_color_and_depth(
        color,
        depth,
        depth_scale=depth_scale,
        depth_trunc=depth_trunc,
        convert_rgb_to_intensity=True,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--depth-trunc", type=float, default=3.0)
    parser.add_argument("--max-translation-m", type=float, default=0.25)
    parser.add_argument("--max-rotation-deg", type=float, default=25.0)
    args = parser.parse_args()

    manifest = json.loads((args.dataset / "manifest.json").read_text(encoding="utf-8"))
    color_info = manifest["camera_info"]["/camera/color/camera_info"]
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        color_info["width"],
        color_info["height"],
        color_info["k"][0],
        color_info["k"][4],
        color_info["k"][2],
        color_info["k"][5],
    )
    frames = manifest["frames"]
    depth_scale = 1000.0 if manifest["depth_unit"] == "millimeter_uint16" else 1.0
    option = o3d.pipelines.odometry.OdometryOption()
    jacobian = o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm()

    world_from_camera = np.eye(4)
    poses = [{"index": 0, "world_from_camera": world_from_camera.tolist()}]
    pairs = []
    previous = load_rgbd(args.dataset, frames[0], depth_scale, args.depth_trunc)
    valid = True
    for index in range(1, len(frames)):
        current = load_rgbd(args.dataset, frames[index], depth_scale, args.depth_trunc)
        success, source_to_target, information = o3d.pipelines.odometry.compute_rgbd_odometry(
            previous, current, intrinsic, np.eye(4), jacobian, option
        )
        rotation_degrees = float(
            np.degrees(
                np.arccos(
                    np.clip((np.trace(source_to_target[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
                )
            )
        )
        translation_m = float(np.linalg.norm(source_to_target[:3, 3]))
        pair_ok = bool(
            success
            and translation_m <= args.max_translation_m
            and rotation_degrees <= args.max_rotation_deg
        )
        if pair_ok:
            world_from_camera = world_from_camera @ np.linalg.inv(source_to_target)
        else:
            valid = False
        pairs.append(
            {
                "source_index": index - 1,
                "target_index": index,
                "odometry_success": bool(success),
                "pair_ok": pair_ok,
                "translation_m": translation_m,
                "rotation_deg": rotation_degrees,
                "information_trace": float(np.trace(information)),
                "source_to_target": source_to_target.tolist(),
            }
        )
        poses.append({"index": index, "world_from_camera": world_from_camera.tolist(), "updated": pair_ok})
        previous = current
        print(f"{index}/{len(frames)-1}: success={success} ok={pair_ok} t={translation_m:.4f}m r={rotation_degrees:.2f}deg", flush=True)

    accepted = [pair for pair in pairs if pair["pair_ok"]]
    result = {
        "schema": "soarm.rgbd_odometry.v1",
        "source_manifest": str(args.dataset / "manifest.json"),
        "trajectory_valid": valid,
        "method": "Open3D RGBDOdometryJacobianFromHybridTerm",
        "world_frame": "first_color_camera",
        "pose_convention": "world_from_camera",
        "quality": {
            "pair_count": len(pairs),
            "accepted_pairs": len(accepted),
            "rejected_pairs": len(pairs) - len(accepted),
            "max_translation_m": max(pair["translation_m"] for pair in pairs),
            "max_rotation_deg": max(pair["rotation_deg"] for pair in pairs),
        },
        "pairs": pairs,
        "poses": poses,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result["quality"], indent=2))
    if not valid:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
