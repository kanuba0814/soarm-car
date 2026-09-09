#!/usr/bin/env python3
"""Reload a companion pilot and perform projective cross-view RGB-D diagnostics."""
import argparse
import json
import math
import struct
import zlib
from pathlib import Path

import numpy as np


def png(path, array):
    height, width, channels = array.shape
    color = 2 if channels == 3 else 0
    raw = b"".join(b"\0" + row.tobytes() for row in array)
    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xffffffff)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, color, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def camera_pose(record):
    x, y, z = record["observed_model_pose"]["translation_m"]
    yaw = record["observed_model_pose"]["yaw_rad"]
    # Model -> base (+0.0763m) -> camera_link (+0.180, 0, +0.150), all before world yaw.
    return np.array([x + .180 * math.cos(yaw), y + .180 * math.sin(yaw), z + .0763 + .150]), yaw


def to_world(points, pose, yaw):
    # Gazebo camera optical convention: optical +Z -> base +X, +X -> base -Y, +Y -> base -Z.
    body = np.stack([points[:, 2], -points[:, 0], -points[:, 1]], axis=1)
    c, s = math.cos(yaw), math.sin(yaw)
    world = body @ np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    return world + pose


def from_world(points, pose, yaw):
    local = points - pose
    c, s = math.cos(yaw), math.sin(yaw)
    body = local @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return np.stack([-body[:, 1], -body[:, 2], body[:, 0]], axis=1)


def load_frame(run, record):
    root = run / "frames" / record["id"]
    depth = np.load(root / "depth_m_float32.npy")
    invalid = np.load(root / "depth_invalid_mask.npy")
    calibration = json.loads((root / "camera_info.json").read_text())
    if depth.dtype != np.float32 or depth.shape != invalid.shape or calibration["depth_unit"] != "m":
        raise RuntimeError(f"reload invariant failed for {record['id']}")
    return depth, invalid, calibration


def point_cloud(depth, invalid, calibration, stride=4):
    fy, fx = calibration["k"][4], calibration["k"][0]
    cx, cy = calibration["k"][2], calibration["k"][5]
    v, u = np.indices(depth.shape)
    keep = (~invalid) & np.isfinite(depth) & (depth > 0) & ((u % stride) == 0) & ((v % stride) == 0)
    z = depth[keep]
    return np.column_stack(((u[keep] - cx) * z / fx, (v[keep] - cy) * z / fy, z))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise SystemExit(f"Refusing to overwrite output directory: {args.output_dir}")
    capture = json.loads((args.input_dir / "capture.json").read_text())
    views = capture["viewpoints"]
    if len(views) < 3:
        raise SystemExit("requires at least three persisted views")
    args.output_dir.mkdir(parents=True)
    loaded = {record["id"]: load_frame(args.input_dir, record) for record in views}
    all_points = []
    for record in views:
        depth, invalid, calibration = loaded[record["id"]]
        all_points.append(to_world(point_cloud(depth, invalid, calibration), *camera_pose(record)))
    cloud = np.concatenate(all_points)
    with (args.output_dir / "combined_world_cloud.xyz").open("w") as stream:
        for point in cloud:
            stream.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
    diagnostics = []
    for index, source in enumerate(views):
        for target in views[index + 1:]:
            source_depth, source_invalid, source_calibration = loaded[source["id"]]
            target_depth, target_invalid, target_calibration = loaded[target["id"]]
            source_points = to_world(point_cloud(source_depth, source_invalid, source_calibration, stride=2), *camera_pose(source))
            projected = from_world(source_points, *camera_pose(target))
            fx, fy, cx, cy = target_calibration["k"][0], target_calibration["k"][4], target_calibration["k"][2], target_calibration["k"][5]
            u = np.rint(projected[:, 0] * fx / projected[:, 2] + cx).astype(int)
            v = np.rint(projected[:, 1] * fy / projected[:, 2] + cy).astype(int)
            in_bounds = (projected[:, 2] > .1) & (u >= 0) & (u < target_depth.shape[1]) & (v >= 0) & (v < target_depth.shape[0])
            residuals = np.array([])
            if in_bounds.any():
                expected = target_depth[v[in_bounds], u[in_bounds]]
                valid = (~target_invalid[v[in_bounds], u[in_bounds]]) & np.isfinite(expected) & (expected > 0)
                # Projective depth agreement establishes an actual cross-view correspondence.
                residuals = np.abs(projected[in_bounds, 2][valid] - expected[valid])
            name = f"{source['id']}_to_{target['id']}"
            image = np.zeros(target_depth.shape, dtype=np.uint8)
            if residuals.size >= 50:
                image[v[in_bounds][valid], u[in_bounds][valid]] = np.clip(residuals / .10 * 255, 0, 255).astype(np.uint8)
                png(args.output_dir / f"{name}_projective_residual.png", image[:, :, None])
                result = {"pair": [source["id"], target["id"]], "method": "projective_depth_correspondence", "projected_source_points": int(len(source_points)), "valid_correspondences": int(residuals.size), "coverage_ratio": float(residuals.size / len(source_points)), "median_m": float(np.median(residuals)), "p95_m": float(np.percentile(residuals, 95)), "threshold_median_le_0.03": bool(np.median(residuals) <= .03), "threshold_p95_le_0.10": bool(np.percentile(residuals, 95) <= .10)}
            else:
                result = {"pair": [source["id"], target["id"]], "method": "projective_depth_correspondence", "projected_source_points": int(len(source_points)), "valid_correspondences": int(residuals.size), "coverage_ratio": float(residuals.size / len(source_points)), "status": "NOT_VERIFIED", "reason": "fewer than 50 projective depth correspondences; no residual threshold asserted"}
            diagnostics.append(result)
    report = {"input": str(args.input_dir), "reload_invariants": "all persisted depth arrays are float32 meters with separately persisted invalid masks and readable complete CameraInfo", "coordinate_convention": "model->base z=.0763, base->camera (.180,0,.150), optical +Z forward/+X right/+Y down", "cross_view": diagnostics, "warning": "Pose and camera-axis conventions are independently recorded simulator observations/declared SDF mounting; this diagnostic is not navigation or method accuracy."}
    (args.output_dir / "geometry.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"views": len(views), "pairs": len(diagnostics), "verified_pairs": sum("median_m" in item for item in diagnostics)}))


if __name__ == "__main__":
    main()
