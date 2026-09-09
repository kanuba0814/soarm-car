#!/usr/bin/env python3
"""Reload persisted RGB-D views and report directional visibility diagnostics, not accuracy."""
import argparse
import json
import math
import struct
import zlib
from pathlib import Path

import numpy as np

TOLERANCE_M = 0.10
MIN_DIAGNOSTIC_PIXELS = 50


def png(path, array):
    height, width, channels = array.shape
    color = 2 if channels == 3 else 0
    raw = b"".join(b"\0" + row.tobytes() for row in array)
    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xffffffff)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, color, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def quaternion_matrix(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0:
        raise ValueError("zero observed pose quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)],
        [2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)],
        [2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)],
    ])


def camera_transform(record):
    if "native_pose_before_frame" in record:
        pose = record["native_pose_before_frame"]["entity"]
        translation = [pose["position"].get(axis, 0.0) for axis in ("x", "y", "z")]
        quaternion = [pose["orientation"].get(axis, 0.0) for axis in ("x", "y", "z", "w")]
    else:
        pose = record["observed_model_pose"]
        translation = pose["translation_m"]
        quaternion = pose["quaternion_xyzw"]
    rotation = quaternion_matrix(*quaternion).T
    # This declared SDF chain is a hypothesis recorded for audit, not optimized from residuals.
    origin = np.asarray(translation, dtype=np.float64)
    camera_origin = origin + rotation @ np.array([.180, 0.0, .0763 + .150])
    # Optical +Z -> base +X, optical +X -> base -Y, optical +Y -> base -Z.
    optical_to_base_rows = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
    return camera_origin, rotation @ optical_to_base_rows.T


def to_world(points, transform):
    origin, rotation = transform
    return points @ rotation.T + origin


def from_world(points, transform):
    origin, rotation = transform
    return (points - origin) @ rotation


def load_frame(run, record):
    root = run / "frames" / record["id"]
    depth = np.load(root / "depth_m_float32.npy")
    invalid = np.load(root / "depth_invalid_mask.npy")
    calibration = json.loads((root / "camera_info.json").read_text())
    if depth.dtype != np.float32 or depth.shape != invalid.shape or calibration["depth_unit"] != "m":
        raise RuntimeError(f"reload invariant failed for {record['id']}")
    return depth, invalid, calibration


def source_points(depth, invalid, calibration, stride=2):
    fy, fx = calibration["k"][4], calibration["k"][0]
    cx, cy = calibration["k"][2], calibration["k"][5]
    v, u = np.indices(depth.shape)
    valid = (~invalid) & np.isfinite(depth) & (depth > 0) & ((u % stride) == 0) & ((v % stride) == 0)
    z = depth[valid].astype(np.float64)
    return np.column_stack(((u[valid] - cx) * z / fx, (v[valid] - cy) * z / fy, z))


def z_buffer_diagnostic(source, target, loaded):
    source_depth, source_invalid, source_calibration = loaded[source["id"]]
    target_depth, target_invalid, target_calibration = loaded[target["id"]]
    camera_points = from_world(to_world(source_points(source_depth, source_invalid, source_calibration), camera_transform(source)), camera_transform(target))
    finite = np.isfinite(camera_points).all(axis=1) & (camera_points[:, 2] > 0.1)
    valid_points = camera_points[finite]
    fx, fy, cx, cy = target_calibration["k"][0], target_calibration["k"][4], target_calibration["k"][2], target_calibration["k"][5]
    u = np.rint(valid_points[:, 0] * fx / valid_points[:, 2] + cx).astype(np.int64)
    v = np.rint(valid_points[:, 1] * fy / valid_points[:, 2] + cy).astype(np.int64)
    in_bounds = (u >= 0) & (u < target_depth.shape[1]) & (v >= 0) & (v < target_depth.shape[0])
    u, v, projected_z = u[in_bounds], v[in_bounds], valid_points[in_bounds, 2]
    pixel_ids = v * target_depth.shape[1] + u
    # Stable nearest-depth z-buffer: source points behind another source cannot claim a target pixel.
    order = np.lexsort((np.arange(len(projected_z)), projected_z, pixel_ids))
    unique = np.empty(len(order), dtype=bool)
    unique[0:] = True
    if len(order):
        unique[1:] = pixel_ids[order][1:] != pixel_ids[order][:-1]
    selected = order[unique]
    u, v, projected_z = u[selected], v[selected], projected_z[selected]
    target_z = target_depth[v, u].astype(np.float64)
    target_valid = (~target_invalid[v, u]) & np.isfinite(target_z) & (target_z > 0)
    signed = projected_z[target_valid] - target_z[target_valid]
    behind = signed > TOLERANCE_M
    foreground = signed < -TOLERANCE_M
    compatible = ~(behind | foreground)
    residual = np.abs(signed)
    image = np.zeros(target_depth.shape, dtype=np.uint8)
    if signed.size:
        image[v[target_valid][behind], u[target_valid][behind]] = 255
        image[v[target_valid][foreground], u[target_valid][foreground]] = 128
        image[v[target_valid][compatible], u[target_valid][compatible]] = 64
    result = {
        "pair": [source["id"], target["id"]],
        "method": "directed_projective_zbuffer_visibility_diagnostic",
        "tolerance_m": TOLERANCE_M,
        "source_sampled_points": int(len(camera_points)),
        "rejected_nonfinite_or_behind_camera": int((~finite).sum()),
        "in_bounds_projected_points": int(len(pixel_ids)),
        "unique_target_pixels_after_nearest_source_zbuffer": int(len(projected_z)),
        "target_invalid_or_nonpositive": int((~target_valid).sum()),
        "behind_nearer_target_surface": int(behind.sum()),
        "foreground_of_target_surface": int(foreground.sum()),
        "depth_compatible_descriptive_only": int(compatible.sum()),
        "supported_target_coverage_ratio": float(target_valid.sum() / len(camera_points)),
        "agreement_is_not_accuracy_proof": True,
    }
    if signed.size >= MIN_DIAGNOSTIC_PIXELS:
        result.update({
            "signed_residual_median_m": float(np.median(signed)),
            "absolute_residual_median_m": float(np.median(residual)),
            "absolute_residual_p95_m": float(np.percentile(residual, 95)),
        })
    else:
        result.update({"status": "NOT_VERIFIED", "reason": "fewer than 50 target-valid unique z-buffer pixels"})
    return result, image


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
    diagnostics = []
    for source in views:
        for target in views:
            if source["id"] == target["id"]:
                continue
            result, image = z_buffer_diagnostic(source, target, loaded)
            png(args.output_dir / f"{source['id']}_to_{target['id']}_visibility.png", image[:, :, None])
            diagnostics.append(result)
    report = {
        "input": str(args.input_dir),
        "reload_invariants": "float32 metric depth, invalid masks, and complete CameraInfo reloaded from persisted artifacts",
        "transform_hypothesis": "full observed quaternion; declared model->base z=.0763, base->camera (.180,0,.150); optical +Z forward/+X right/+Y down",
        "limitations": ["dynamic_pose parent/world semantics and axial-depth convention remain separately unverified", "visibility categories are conditional on the transform hypothesis", "depth-compatible counts are descriptive and do not certify accuracy or same-surface identity"],
        "directed_pairs": diagnostics,
    }
    (args.output_dir / "geometry.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"views": len(views), "directed_pairs": len(diagnostics), "target_valid_unique_pixels": sum(item["behind_nearer_target_surface"] + item["foreground_of_target_surface"] + item["depth_compatible_descriptive_only"] for item in diagnostics)}))


if __name__ == "__main__":
    main()
