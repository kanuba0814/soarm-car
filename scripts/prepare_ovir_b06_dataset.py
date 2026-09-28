#!/usr/bin/env python3
"""Prepare B06 for OVIR-3D using measured RGB-D and estimated camera poses."""

import argparse
import json
from pathlib import Path

import numpy as np
import open3d as o3d


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("odometry", type=Path)
    parser.add_argument("detic_output", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--voxel-size", type=float, default=0.01)
    args = parser.parse_args()

    manifest = json.loads((args.dataset / "manifest.json").read_text(encoding="utf-8"))
    odometry = json.loads(args.odometry.read_text(encoding="utf-8"))
    if not odometry["trajectory_valid"]:
        raise RuntimeError("Refusing to prepare OVIR data from an invalid trajectory")
    if len(odometry["poses"]) != len(manifest["frames"]):
        raise RuntimeError("Pose count does not match frame count")

    for name in ("color", "depth", "poses"):
        (args.output / name).mkdir(parents=True, exist_ok=True)
    output_instances = args.output / "detic_output" / "imagenet21k-0.3" / "instances"
    output_instances.mkdir(parents=True, exist_ok=True)

    color_info = manifest["camera_info"]["/camera/color/camera_info"]
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        color_info["width"], color_info["height"], color_info["k"][0],
        color_info["k"][4], color_info["k"][2], color_info["k"][5]
    )
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=args.voxel_size,
        sdf_trunc=args.voxel_size * 4,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )

    for frame, pose_item in zip(manifest["frames"], odometry["poses"]):
        frame_id = f"{frame['index']:06d}"
        color_source = (args.dataset / frame["rgb"]).resolve()
        depth_source = (args.dataset / frame["depth"]).resolve()
        color_link = args.output / "color" / f"{frame_id}-color.jpg"
        depth_link = args.output / "depth" / f"{frame_id}-depth.png"
        instance_link = output_instances / f"{frame_id}-color.pkl"
        for link, source in (
            (color_link, color_source),
            (depth_link, depth_source),
            (instance_link, (args.detic_output / f"{frame_id}.pkl").resolve()),
        ):
            if link.exists() or link.is_symlink():
                link.unlink()
            link.symlink_to(source)

        world_from_camera = np.asarray(pose_item["world_from_camera"], dtype=float)
        np.savetxt(args.output / "poses" / f"{frame_id}-pose.txt", world_from_camera)
        color = o3d.io.read_image(str(color_source))
        depth = o3d.io.read_image(str(depth_source))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_scale=1000.0, depth_trunc=3.0,
            convert_rgb_to_intensity=False,
        )
        volume.integrate(rgbd, intrinsic, np.linalg.inv(world_from_camera))

    config = {
        "id": "B06",
        "im_w": color_info["width"],
        "im_h": color_info["height"],
        "depth_scale": 1000,
        "cam_intr": np.asarray(color_info["k"]).reshape(3, 3).tolist(),
        "pose_source": str(args.odometry),
        "rgb_depth_registration": "assumed_from_equal_intrinsics_and_driver_tf; not independently calibrated",
    }
    (args.output / "config.json").write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    point_cloud = volume.extract_point_cloud().voxel_down_sample(args.voxel_size)
    output_cloud = args.output / f"scan-{args.voxel_size:.3f}.pcd"
    if not o3d.io.write_point_cloud(str(output_cloud), point_cloud):
        raise RuntimeError(f"Failed to write {output_cloud}")
    summary = {
        "frames": len(manifest["frames"]),
        "points": len(point_cloud.points),
        "point_cloud": str(output_cloud),
    }
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
