"""Validated import and replay-time queries for archived RGB-D observations."""
import hashlib
import json
import math
import os
import tempfile
from pathlib import Path

import numpy as np


SCHEMA_VERSION = 1
DEFAULT_POLICY = {
    "source_clock_id": "public-vlm-grid-dev-v1-source-stamp-ns",
    "ttl_ns": 1_500_000_000,
    "category_aliases": {
        "chair": "chair",
        "椅子": "chair",
        "diningtable": "dining table",
        "dining table": "dining table",
        "餐桌": "dining table",
    },
}


class ValidationError(ValueError):
    """Raised when an archive cannot support a trustworthy import."""


def _read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _finite(values):
    return all(math.isfinite(float(value)) for value in values)


def _validate_manifest(public_dir, manifest):
    if manifest.get("scope") != "PUBLIC_RGBD_VLM_GRID_DEV_INPUT":
        raise ValidationError("unsupported public-input manifest scope")
    files = manifest.get("files")
    if not isinstance(files, dict) or len(files) != 19:
        raise ValidationError("manifest must declare exactly 19 files")
    for relative, expected in files.items():
        candidate = public_dir / relative
        if not candidate.is_file():
            raise ValidationError(f"manifest file missing: {relative}")
        if _sha256(candidate) != expected:
            raise ValidationError(f"hash mismatch: {relative}")


def _camera_transform(self_pose, mounting):
    orientation = self_pose["entity"]["orientation"]
    quaternion = np.asarray([orientation[key] for key in ("x", "y", "z", "w")], dtype=float)
    translation = np.asarray(mounting["base_to_camera_translation_m"], dtype=float)
    optical_to_base = np.asarray(mounting["optical_to_base_rows"], dtype=float)
    position = np.asarray([self_pose["entity"]["position"][key] for key in ("x", "y", "z")], dtype=float)
    if not _finite(quaternion) or not _finite(translation) or not _finite(optical_to_base.ravel()) or not _finite(position):
        raise ValidationError("nonfinite calibration")
    norm = np.linalg.norm(quaternion)
    if not math.isfinite(float(norm)) or norm == 0:
        raise ValidationError("invalid calibration quaternion")
    x, y, z, w = quaternion / norm
    body_to_world = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)],
        [2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)],
        [2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)],
    ]).T
    return position + body_to_world @ translation, body_to_world @ optical_to_base.T


def _geometry(depth, invalid, info, transform, box):
    if depth.shape != invalid.shape or depth.ndim != 2:
        raise ValidationError("invalid depth or invalid-depth mask shape")
    height, width = depth.shape
    if info.get("width") != width or info.get("height") != height:
        raise ValidationError("camera dimensions do not match depth")
    intrinsics = np.asarray(info.get("k", []), dtype=float)
    if intrinsics.shape != (9,) or not _finite(intrinsics) or intrinsics[0] <= 0 or intrinsics[4] <= 0:
        raise ValidationError("invalid calibration intrinsics")
    if not np.isfinite(depth).any():
        raise ValidationError("nonfinite depth with no usable values")
    left, top, right, bottom = (int(round(value)) for value in box)
    left, right = max(0, left), min(width, right)
    top, bottom = max(0, top), min(height, bottom)
    if right <= left or bottom <= top:
        return {"status": "rejected", "reason": "empty_clipped_box"}
    sample = depth[top:bottom, left:right]
    valid = np.isfinite(sample) & (sample > 0) & (~invalid[top:bottom, left:right])
    count = int(valid.sum())
    if count < 20:
        return {"status": "rejected", "reason": "insufficient_valid_depth", "valid_pixels": count}
    rows, columns = np.argwhere(valid).T
    rows, columns = rows + top, columns + left
    z = depth[rows, columns].astype(float)
    fx, fy, cx, cy = intrinsics[0], intrinsics[4], intrinsics[2], intrinsics[5]
    camera = np.column_stack(((columns - cx) * z / fx, (rows - cy) * z / fy, z))
    origin, rotation = transform
    world = camera @ rotation.T + origin
    xy = np.median(world[:, :2], axis=0)
    return {
        "status": "ok",
        "calibration_self_pose_xy_m": [float(xy[0]), float(xy[1])],
        "depth_median_m": float(np.median(z)),
        "valid_pixels": count,
        "valid_ratio": float(count / sample.size),
        "confidence": "geometry_supported" if count >= 100 else "geometry_limited",
    }


def _fingerprint(record):
    canonical = json.dumps(record, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(canonical.encode("ascii")).hexdigest()


def _append_revision(store, record):
    """Append a distinct revision without allowing source-time reordering to replace it."""
    fingerprint = _fingerprint(record)
    bucket = store["records"].setdefault(record["candidate_id"], {"revisions": [], "conflict_open": False})
    if any(item["fingerprint"] == fingerprint for item in bucket["revisions"]):
        return False
    bucket["revisions"].append({"fingerprint": fingerprint, "record": record})
    bucket["revisions"].sort(key=lambda item: item["record"]["source_stamp_ns"], reverse=True)
    newest_stamp = bucket["revisions"][0]["record"]["source_stamp_ns"]
    bucket["conflict_open"] = bucket["conflict_open"] or sum(
        item["record"]["source_stamp_ns"] == newest_stamp for item in bucket["revisions"]
    ) > 1
    return True


def _atomic_write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(value, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        os.unlink(temporary)
        raise


def load_store(store_path):
    data = _read_json(store_path)
    if data.get("schema_version") != SCHEMA_VERSION:
        raise ValidationError("unsupported semantic-memory store schema")
    return data


def import_public_archive(public_dir, store_path, policy=None):
    """Validate the archive and atomically append observation-local revisions."""
    public_dir, store_path = Path(public_dir), Path(store_path)
    policy = {**DEFAULT_POLICY, **(policy or {})}
    manifest = _read_json(public_dir / "manifest.json")
    _validate_manifest(public_dir, manifest)
    calibration = _read_json(public_dir / "calibration_selfpose.json")["observations"]
    gates = _read_json(public_dir / "gates.json")["observations"]
    detections = _read_json(public_dir / "detections.json")["observations"]
    existing = load_store(store_path) if store_path.exists() else {
        "schema_version": SCHEMA_VERSION, "policy": policy, "records": {}, "imports": []
    }
    if existing["policy"] != policy:
        raise ValidationError("store policy differs from importer policy")
    imported = []
    for observation_id in manifest["observations"]:
        observation = calibration.get(observation_id)
        gate = gates.get(observation_id)
        if not observation or not gate or not gate.get("source_time_valid") or not gate.get("eligible"):
            raise ValidationError(f"invalid source gate: {observation_id}")
        stamp = observation["self_pose"].get("source_stamp_ns")
        if not isinstance(stamp, int) or stamp < 0:
            raise ValidationError(f"invalid source stamp: {observation_id}")
        transform = _camera_transform(observation["self_pose"], observation["camera_mounting"])
        root = public_dir / observation_id
        depth = np.load(root / "depth_m_float32.npy", allow_pickle=False)
        invalid = np.load(root / "depth_invalid_mask.npy", allow_pickle=False).astype(bool)
        info = _read_json(root / "camera_info.json")
        rgb_relative = f"{observation_id}/rgb.png"
        for detection in detections.get(observation_id, []):
            geometry = _geometry(depth, invalid, info, transform, detection["box_xyxy"])
            candidate_id = f"{observation_id}:{detection['detection_index']}"
            record = {
                "candidate_id": candidate_id,
                "observation_id": observation_id,
                "detection_index": detection["detection_index"],
                "label": detection["label"],
                "detector_confidence": float(detection["confidence"]),
                "box_xyxy": [float(value) for value in detection["box_xyxy"]],
                "source_clock_id": policy["source_clock_id"],
                "source_stamp_ns": stamp,
                "geometry": geometry,
                "evidence": {
                    "rgb": {"path": rgb_relative, "sha256": manifest["files"][rgb_relative]},
                    "depth": {"path": f"{observation_id}/depth_m_float32.npy", "sha256": manifest["files"][f"{observation_id}/depth_m_float32.npy"]},
                    "invalid_depth_mask": {"path": f"{observation_id}/depth_invalid_mask.npy", "sha256": manifest["files"][f"{observation_id}/depth_invalid_mask.npy"]},
                    "camera_info": {"path": f"{observation_id}/camera_info.json", "sha256": manifest["files"][f"{observation_id}/camera_info.json"]},
                    "calibration": {"path": "calibration_selfpose.json", "sha256": manifest["files"]["calibration_selfpose.json"]},
                },
            }
            if _append_revision(existing, record):
                imported.append(candidate_id)
    archive_hash = _sha256(public_dir / "manifest.json")
    if archive_hash not in existing["imports"]:
        existing["imports"].append(archive_hash)
    _atomic_write(store_path, existing)
    return {"imported_revisions": len(imported), "candidate_ids": sorted(set(imported)), "records": len(existing["records"])}


def query(store_path, category, replay_time_ns, source_clock_id):
    store = load_store(store_path)
    canonical = store["policy"]["category_aliases"].get(category.strip().lower())
    response = {"motion_authorized": False, "category": category, "canonical_category": canonical}
    if canonical is None:
        return {**response, "status": "not_found", "reason": "unknown_category_alias", "candidates": []}
    candidates = []
    unconfirmed = []
    expired = []
    for candidate_id, bucket in store["records"].items():
        revisions = bucket["revisions"]
        current = revisions[0]["record"]
        if current["label"] != canonical:
            continue
        if bucket["conflict_open"]:
            unconfirmed.append((candidate_id, "conflicting_revisions", revisions))
        elif source_clock_id != current["source_clock_id"]:
            unconfirmed.append((candidate_id, "incompatible_clock", revisions))
        elif replay_time_ns < current["source_stamp_ns"]:
            unconfirmed.append((candidate_id, "future_source_stamp", revisions))
        elif replay_time_ns - current["source_stamp_ns"] > store["policy"]["ttl_ns"]:
            expired.append((candidate_id, revisions))
        else:
            candidates.append(current)
    if unconfirmed:
        return {**response, "status": "needs_confirmation", "reason": unconfirmed[0][1], "candidates": [item[0] for item in unconfirmed], "revision_counts": {item[0]: len(item[2]) for item in unconfirmed}}
    if not candidates and expired:
        return {**response, "status": "needs_confirmation", "reason": "expired", "candidates": [item[0] for item in expired], "revision_counts": {item[0]: len(item[1]) for item in expired}}
    if not candidates:
        return {**response, "status": "not_found", "reason": "no_observation_local_candidate", "candidates": []}
    if len(candidates) > 1:
        return {**response, "status": "needs_confirmation", "reason": "multiple_unmerged_observation_local_candidates", "candidates": [item["candidate_id"] for item in candidates]}
    return {**response, "status": "candidates", "reason": "fresh_single_observation_local_candidate", "candidates": candidates}
