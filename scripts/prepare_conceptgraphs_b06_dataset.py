#!/usr/bin/env python3
"""Prepare the B06 export for ConceptGraphs' official AzureKinectDataset loader.

ConceptGraphs only accepts JPEG colour files in that loader.  This adapter
therefore transcodes the supplied PNG colour images into its own output tree,
keeps 16-bit depth as PNG, and writes the measured odometry as the loader's
``poses_global_dvo.txt`` format.  It never estimates or fills in poses.
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import cv2


def reset_directory(path: Path) -> None:
    if path.exists():
        shutil.rmtree(path)
    path.mkdir(parents=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--odometry", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    dataset = args.dataset.resolve()
    output = args.output.resolve()
    manifest = json.loads((dataset / "manifest.json").read_text(encoding="utf-8"))
    odometry = json.loads(args.odometry.read_text(encoding="utf-8"))
    if not odometry.get("trajectory_valid"):
        raise ValueError("refusing to prepare ConceptGraphs input from an invalid trajectory")
    frames = manifest["frames"]
    poses = odometry["poses"]
    if len(frames) != len(poses):
        raise ValueError(f"frame/pose count mismatch: {len(frames)} != {len(poses)}")

    camera = manifest["camera_info"]["/camera/color/camera_info"]
    if camera["width"] != 640 or camera["height"] != 480:
        raise ValueError("B06 adapter expects the supplied 640x480 colour camera")

    sequence = output / "B06"
    reset_directory(sequence)
    (sequence / "color").mkdir()
    (sequence / "depth").mkdir()
    pose_lines = []
    for frame, pose_entry in zip(frames, poses):
        index = int(frame["index"])
        if index != int(pose_entry["index"]):
            raise ValueError(f"frame/pose index mismatch for {index}")
        color = cv2.imread(str(dataset / frame["rgb"]), cv2.IMREAD_COLOR)
        depth_path = dataset / frame["depth"]
        depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        if color is None or color.shape[:2] != (480, 640):
            raise ValueError(f"invalid colour image for frame {index}")
        if depth is None or depth.shape != (480, 640) or depth.dtype != "uint16":
            raise ValueError(f"invalid uint16 depth image for frame {index}")
        frame_id = f"{index:06d}"
        if not cv2.imwrite(str(sequence / "color" / f"{frame_id}.jpg"), color):
            raise RuntimeError(f"failed to write colour frame {index}")
        depth_link = sequence / "depth" / f"{frame_id}.png"
        depth_link.symlink_to(os.path.relpath(depth_path, depth_link.parent))
        matrix = pose_entry["world_from_camera"]
        if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
            raise ValueError(f"invalid 4x4 pose for frame {index}")
        pose_lines.append(" ".join(str(float(value)) for row in matrix for value in row))

    (sequence / "poses_global_dvo.txt").write_text("\n".join(pose_lines) + "\n", encoding="utf-8")
    config = {
        "dataset_name": "azurekinect",
        "camera_params": {
            "image_height": int(camera["height"]),
            "image_width": int(camera["width"]),
            "fx": float(camera["k"][0]), "fy": float(camera["k"][4]),
            "cx": float(camera["k"][2]), "cy": float(camera["k"][5]),
            "png_depth_scale": 1000.0,
        },
    }
    (output / "B06-azurekinect.yaml").write_text(
        "dataset_name: azurekinect\ncamera_params:\n"
        f"  image_height: {config['camera_params']['image_height']}\n"
        f"  image_width: {config['camera_params']['image_width']}\n"
        f"  fx: {config['camera_params']['fx']}\n  fy: {config['camera_params']['fy']}\n"
        f"  cx: {config['camera_params']['cx']}\n  cy: {config['camera_params']['cy']}\n"
        "  png_depth_scale: 1000.0\n",
        encoding="utf-8",
    )
    metadata = {
        "schema": "soarm.conceptgraphs_b06_adapter.v1",
        "official_dataset_class": "AzureKinectDataset",
        "frames": len(frames),
        "intrinsics": config["camera_params"],
        "pose_convention": odometry["pose_convention"],
        "source_manifest": str((dataset / "manifest.json").resolve()),
        "source_odometry": str(args.odometry.resolve()),
        "rgb_depth_registration": "not independently verified",
        "colour_conversion": "PNG source transcoded to JPEG because AzureKinectDataset glob is color/*.jpg",
    }
    (output / "B06-adapter.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
