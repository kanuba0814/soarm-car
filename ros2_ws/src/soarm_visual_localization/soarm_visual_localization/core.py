"""Dependency-light guarded RGB-D geometry for offline and shadow use."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import hashlib
import json
from pathlib import Path
from typing import Iterable

import numpy as np


class VisualState(str, Enum):
    INITIALIZING = "INITIALIZING"
    TRACKING = "TRACKING"
    DEGRADED = "DEGRADED"
    RELOCALIZING = "RELOCALIZING"
    LOST = "LOST"


@dataclass(frozen=True)
class CameraProfile:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale: float = 1.0
    depth_min_m: float = 0.15
    depth_max_m: float = 6.0
    registration_verified: bool = False

    @classmethod
    def from_json(cls, path: str | Path) -> "CameraProfile":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(**data)


@dataclass(frozen=True)
class RGBDFrame:
    color: np.ndarray
    depth: np.ndarray
    stamp: float
    frame_id: str
    color_stamp: float | None = None
    depth_stamp: float | None = None
    camera_info_stamp: float | None = None


class RGBDInputAdapter:
    """Normalize camera streams before they enter a visual estimator."""

    def __init__(self, profile: CameraProfile, *, max_sync_skew_s: float = 0.05,
                 min_valid_depth_ratio: float = 0.05):
        self.profile = profile
        self.max_sync_skew_s = max_sync_skew_s
        self.min_valid_depth_ratio = min_valid_depth_ratio

    def make_frame(self, color: np.ndarray, depth: np.ndarray, stamp: float, frame_id: str,
                   *, color_stamp: float | None = None,
                   depth_stamp: float | None = None,
                   camera_info_stamp: float | None = None) -> RGBDFrame:
        if color.ndim not in (2, 3) or color.shape[:2] != (self.profile.height, self.profile.width):
            raise ValueError("color shape does not match camera profile")
        if depth.ndim != 2 or depth.shape != (self.profile.height, self.profile.width):
            raise ValueError("depth shape does not match camera profile")
        if not frame_id:
            raise ValueError("camera frame_id is empty")
        stamps = [value for value in (color_stamp, depth_stamp, camera_info_stamp)
                  if value is not None]
        if not np.isfinite(stamp) or any(not np.isfinite(value) for value in stamps):
            raise ValueError("RGB-D timestamp is not finite")
        if stamps and max(stamps) - min(stamps) > self.max_sync_skew_s:
            raise ValueError("RGB-D timestamp skew exceeds limit")
        valid_ratio = float(np.count_nonzero(_depth_valid_mask(depth, self.profile))) / depth.size
        if valid_ratio < self.min_valid_depth_ratio:
            raise ValueError("RGB-D valid depth ratio is below limit")
        return RGBDFrame(color, depth, float(stamp), str(frame_id),
                         color_stamp=color_stamp, depth_stamp=depth_stamp,
                         camera_info_stamp=camera_info_stamp)


@dataclass
class VisualMeasurement:
    state: VisualState
    transform: np.ndarray = field(default_factory=lambda: np.eye(4))
    inliers: int = 0
    rmse_m: float = float("inf")
    valid_depth_ratio: float = 0.0
    translation_m: float = 0.0
    rotation_rad: float = 0.0
    observability_ratio: float = 0.0
    reason: str = ""

    @property
    def accepted(self) -> bool:
        return self.state == VisualState.TRACKING


def depth_to_points(depth: np.ndarray, profile: CameraProfile) -> tuple[np.ndarray, np.ndarray]:
    """Back-project registered depth pixels; returns points and integer pixel indices."""
    if depth.ndim != 2 or depth.shape != (profile.height, profile.width):
        raise ValueError("depth shape does not match camera profile")
    values = depth.astype(np.float64) * profile.depth_scale
    ys, xs = np.indices(depth.shape)
    valid = np.isfinite(values) & (values >= profile.depth_min_m) & (values <= profile.depth_max_m)
    z = values[valid]
    x = (xs[valid] - profile.cx) * z / profile.fx
    y = (ys[valid] - profile.cy) * z / profile.fy
    return np.column_stack((x, y, z)), np.column_stack((xs[valid], ys[valid]))


def _depth_points_for_mask(depth: np.ndarray, profile: CameraProfile, valid: np.ndarray) -> np.ndarray:
    values = depth.astype(np.float64) * profile.depth_scale
    ys, xs = np.indices(depth.shape)
    z = values[valid]
    x = (xs[valid] - profile.cx) * z / profile.fx
    y = (ys[valid] - profile.cy) * z / profile.fy
    return np.column_stack((x, y, z))


def _depth_valid_mask(depth: np.ndarray, profile: CameraProfile) -> np.ndarray:
    values = depth.astype(np.float64) * profile.depth_scale
    return np.isfinite(values) & (values >= profile.depth_min_m) & (values <= profile.depth_max_m)


def _kabsch(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    covariance = (source - source_mean).T @ (target - target_mean)
    u, singular, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    translation = target_mean - rotation @ source_mean
    return rotation, translation, singular


def _robust_kabsch(source: np.ndarray, target: np.ndarray, *, trim_fraction: float = 0.85,
                   iterations: int = 3) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Fit a rigid transform while trimming the largest geometric residuals."""
    if not 0.5 <= trim_fraction <= 1.0 or iterations < 1:
        raise ValueError("invalid robust Kabsch options")
    active = np.arange(len(source))
    rotation = np.eye(3)
    translation = np.zeros(3)
    singular = np.zeros(3)
    for _ in range(iterations):
        rotation, translation, singular = _kabsch(source[active], target[active])
        residual = np.linalg.norm(_transform(source, rotation, translation) - target, axis=1)
        keep = max(3, int(np.floor(len(source) * trim_fraction)))
        order = np.argsort(residual, kind="stable")
        active = order[:keep]
    rotation, translation, singular = _kabsch(source[active], target[active])
    residual = np.linalg.norm(_transform(source[active], rotation, translation) - target[active], axis=1)
    return rotation, translation, singular, residual


def _transform(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return points @ rotation.T + translation


class GuardedRGBDOdometry:
    """Small, deterministic RGB-D tracker with explicit rejection semantics.

    This is deliberately a baseline/research implementation. It assumes registered
    depth and uses identical-pixel correspondences for a bounded local update; it
    never consumes LiDAR, wheel odometry, simulator truth, or an external pose prior.
    """

    def __init__(self, profile: CameraProfile, *, min_inliers: int = 80,
                 max_rmse_m: float = 0.035, max_translation_m: float = 0.30,
                 max_rotation_rad: float = 0.45, min_observability: float = 0.02,
                 trim_fraction: float = 0.85):
        if not 0.5 <= trim_fraction <= 1.0:
            raise ValueError("trim_fraction must be between 0.5 and 1.0")
        self.profile = profile
        self.min_inliers = min_inliers
        self.max_rmse_m = max_rmse_m
        self.max_translation_m = max_translation_m
        self.max_rotation_rad = max_rotation_rad
        self.min_observability = min_observability
        self.trim_fraction = trim_fraction
        self.state = VisualState.INITIALIZING
        self.pose = np.eye(4)
        self.previous: RGBDFrame | None = None

    def reset(self) -> None:
        self.state = VisualState.INITIALIZING
        self.pose = np.eye(4)
        self.previous = None

    def update(self, frame: RGBDFrame) -> VisualMeasurement:
        if not self.profile.registration_verified:
            self.state = VisualState.DEGRADED
            return VisualMeasurement(self.state, reason="rgb_depth_registration_unverified")
        if self.previous is None:
            self.previous = frame
            self.state = VisualState.TRACKING
            return VisualMeasurement(self.state, transform=self.pose.copy(), reason="initialized")
        if frame.frame_id != self.previous.frame_id:
            self.state = VisualState.DEGRADED
            return VisualMeasurement(self.state, reason="camera_frame_changed")
        dt = frame.stamp - self.previous.stamp
        if not np.isfinite(dt) or dt <= 0.0 or dt > 0.25:
            self.state = VisualState.DEGRADED
            return VisualMeasurement(self.state, reason="stale_or_nonmonotonic_timestamp")
        old_valid = _depth_valid_mask(self.previous.depth, self.profile)
        new_valid = _depth_valid_mask(frame.depth, self.profile)
        common = old_valid & new_valid
        old = _depth_points_for_mask(self.previous.depth, self.profile, common)
        new = _depth_points_for_mask(frame.depth, self.profile, common)
        count = len(old)
        if count < self.min_inliers:
            self.state = VisualState.LOST
            self.previous = frame
            return VisualMeasurement(self.state, inliers=count, reason="insufficient_valid_depth")
        # Deterministic spatial thinning limits runtime and avoids frame-order bias.
        stride = max(1, count // 1200)
        source, target = old[::stride], new[::stride]
        rotation, translation, singular, residual = _robust_kabsch(
            source, target, trim_fraction=self.trim_fraction)
        rmse = float(np.sqrt(np.mean(residual * residual)))
        scale = float(singular[0]) if singular[0] > 0 else 0.0
        observability = float(singular[-1] / singular[0]) if scale else 0.0
        rotation_rad = float(np.arccos(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)))
        accepted = (len(source) >= self.min_inliers and rmse <= self.max_rmse_m
                    and np.linalg.norm(translation) <= self.max_translation_m
                    and rotation_rad <= self.max_rotation_rad
                    and observability >= self.min_observability)
        if accepted:
            step = np.eye(4)
            step[:3, :3] = rotation
            step[:3, 3] = translation
            self.pose = self.pose @ np.linalg.inv(step)
            self.state = VisualState.TRACKING
            reason = "accepted"
        else:
            self.state = VisualState.DEGRADED if len(source) >= self.min_inliers else VisualState.LOST
            reason = "quality_gate_rejected"
        self.previous = frame
        valid_ratio = float(len(old)) / float(self.profile.width * self.profile.height)
        return VisualMeasurement(self.state, self.pose.copy(), len(source), rmse,
                                 valid_ratio, float(np.linalg.norm(translation)),
                                 rotation_rad, observability, reason)


@dataclass
class Keyframe:
    keyframe_id: int
    stamp: float
    pose: np.ndarray
    points: np.ndarray
    descriptor: str


class VisualMap:
    """Immutable-on-save keyframe map used by the pure-visual relocalizer."""

    schema = "soarm.visual_map.v1"

    def __init__(self, profile: CameraProfile):
        self.profile = profile
        self.keyframes: list[Keyframe] = []

    def add(self, stamp: float, pose: np.ndarray, points: np.ndarray) -> Keyframe:
        if pose.shape != (4, 4) or points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("invalid keyframe geometry")
        digest = hashlib.sha256(np.asarray(points, dtype=np.float32).tobytes()).hexdigest()
        keyframe = Keyframe(len(self.keyframes), stamp, pose.copy(), points.copy(), digest)
        self.keyframes.append(keyframe)
        return keyframe

    def save(self, path: str | Path, *, map_id: str) -> None:
        if not self.keyframes:
            raise ValueError("cannot save empty visual map")
        payload = {
            "schema": self.schema,
            "map_id": map_id,
            "profile": self.profile.__dict__,
            "keyframes": [{"id": k.keyframe_id, "stamp": k.stamp,
                           "pose": k.pose.tolist(), "descriptor": k.descriptor,
                           "point_count": len(k.points)} for k in self.keyframes],
        }
        Path(path).write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    def save_bundle(self, directory: str | Path, *, map_id: str) -> Path:
        if not self.keyframes:
            raise ValueError("cannot save empty visual map")
        root = Path(directory)
        root.mkdir(parents=True, exist_ok=True)
        keyframes = []
        for keyframe in self.keyframes:
            points_path = root / f"keyframe_{keyframe.keyframe_id:06d}.npy"
            np.save(points_path, keyframe.points.astype(np.float32), allow_pickle=False)
            keyframes.append({"id": keyframe.keyframe_id, "stamp": keyframe.stamp,
                              "pose": keyframe.pose.tolist(), "descriptor": keyframe.descriptor,
                              "point_count": len(keyframe.points), "points": points_path.name})
        payload = {"schema": self.schema, "map_id": map_id,
                   "profile": self.profile.__dict__, "keyframes": keyframes}
        manifest = root / "map.json"
        manifest.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        digest = hashlib.sha256()
        for path in sorted(root.glob("keyframe_*.npy")):
            digest.update(path.name.encode("utf-8"))
            digest.update(path.read_bytes())
        digest.update(manifest.read_bytes())
        (root / "bundle.sha256").write_text(digest.hexdigest() + "\n", encoding="utf-8")
        return manifest

    @classmethod
    def load_bundle(cls, directory: str | Path) -> tuple["VisualMap", str]:
        root = Path(directory)
        payload = json.loads((root / "map.json").read_text(encoding="utf-8"))
        if payload.get("schema") != cls.schema:
            raise ValueError("unsupported visual map schema")
        visual_map = cls(CameraProfile(**payload["profile"]))
        for item in payload["keyframes"]:
            points = np.load(root / item["points"], allow_pickle=False)
            visual_map.keyframes.append(Keyframe(
                int(item["id"]), float(item["stamp"]), np.asarray(item["pose"], dtype=float),
                points, str(item["descriptor"])))
        return visual_map, str(payload["map_id"])

    def descriptor_candidates(self, points: np.ndarray, limit: int = 5) -> list[Keyframe]:
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("invalid query geometry")
        query = np.quantile(points, [0.1, 0.5, 0.9], axis=0)
        scored = []
        for keyframe in self.keyframes:
            ref = np.quantile(keyframe.points, [0.1, 0.5, 0.9], axis=0)
            scored.append((float(np.linalg.norm(query - ref)), keyframe))
        return [item[1] for item in sorted(scored, key=lambda item: item[0])[:limit]]

    def relocalize(self, points: np.ndarray, *, max_rmse_m: float = 0.08) -> tuple[Keyframe | None, np.ndarray, float]:
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
            raise ValueError("invalid query geometry")
        best = None
        for keyframe in self.descriptor_candidates(points):
            count = min(len(points), len(keyframe.points))
            if count < 3:
                continue
            source = keyframe.points[:count]
            target = points[:count]
            rotation, translation, _ = _kabsch(source, target)
            predicted = _transform(source, rotation, translation)
            residual = np.linalg.norm(predicted - target, axis=1)
            rmse = float(np.sqrt(np.mean(residual * residual)))
            if best is None or rmse < best[2]:
                step = np.eye(4)
                step[:3, :3] = rotation
                step[:3, 3] = translation
                best = (keyframe, keyframe.pose @ np.linalg.inv(step), rmse)
        if best is None or best[2] > max_rmse_m:
            return None, np.eye(4), float("inf")
        return best

        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
            raise ValueError("invalid query geometry")
        best = None
        for keyframe in self.descriptor_candidates(points):
            count = min(len(points), len(keyframe.points))
            if count < 3:
                continue
            source = keyframe.points[:count]
            target = points[:count]
            rotation, translation, _ = _kabsch(source, target)
            predicted = _transform(source, rotation, translation)
            residual = np.linalg.norm(predicted - target, axis=1)
            rmse = float(np.sqrt(np.mean(residual * residual)))
            if best is None or rmse < best[2]:
                step = np.eye(4)
                step[:3, :3] = rotation
                step[:3, 3] = translation
                best = (keyframe, keyframe.pose @ np.linalg.inv(step), rmse)
        if best is None or best[2] > max_rmse_m:
            return None, np.eye(4), float("inf")
        return best
