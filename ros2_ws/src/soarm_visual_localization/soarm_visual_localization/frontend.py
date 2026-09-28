"""Dependency-light RGB-D feature tracking and geometric odometry."""
from __future__ import annotations

import numpy as np

from .core import (
    CameraProfile,
    RGBDFrame,
    VisualMeasurement,
    VisualState,
    _depth_valid_mask,
    _kabsch,
    _robust_kabsch,
)


def _gray(color: np.ndarray) -> np.ndarray:
    if color.ndim == 2:
        return color.astype(np.float64)
    if color.ndim == 3 and color.shape[2] >= 3:
        return (0.299 * color[..., 0] + 0.587 * color[..., 1] + 0.114 * color[..., 2]).astype(np.float64)
    raise ValueError("unsupported color shape")


def _pixel_points(depth: np.ndarray, pixels: np.ndarray, profile: CameraProfile) -> np.ndarray:
    values = depth[pixels[:, 1], pixels[:, 0]].astype(np.float64) * profile.depth_scale
    x = (pixels[:, 0] - profile.cx) * values / profile.fx
    y = (pixels[:, 1] - profile.cy) * values / profile.fy
    return np.column_stack((x, y, values))


def _ransac_rigid(source: np.ndarray, target: np.ndarray, *, iterations: int = 64,
                  threshold_m: float = 0.04) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Find a rigid 3D correspondence model and return its inlier mask."""
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("source and target must be matching Nx3 arrays")
    if len(source) < 3:
        raise ValueError("at least three correspondences are required")
    if iterations < 1 or threshold_m <= 0.0:
        raise ValueError("invalid RANSAC options")
    rng = np.random.default_rng(0)
    best_mask = np.zeros(len(source), dtype=bool)
    best_error = float("inf")
    for _ in range(iterations):
        sample = rng.choice(len(source), size=3, replace=False)
        try:
            rotation, translation, _ = _kabsch(source[sample], target[sample])
        except np.linalg.LinAlgError:
            continue
        residual = np.linalg.norm(source @ rotation.T + translation - target, axis=1)
        mask = residual <= threshold_m
        error = float(np.sum(np.minimum(residual, threshold_m) ** 2))
        if (int(mask.sum()), -error) > (int(best_mask.sum()), -best_error):
            best_mask, best_error = mask, error
    if best_mask.sum() < 3:
        best_mask[:] = True
    rotation, translation, _, residual = _robust_kabsch(
        source[best_mask], target[best_mask], trim_fraction=1.0)
    refined_residual = np.linalg.norm(
        source @ rotation.T + translation - target, axis=1)
    refined_mask = refined_residual <= threshold_m
    if refined_mask.sum() >= 3:
        best_mask = refined_mask
    return rotation, translation, best_mask


class PatchRGBDOdometry:
    """Small RGB-D front end with deterministic patch matching and fail-closed gates.

    It is intentionally dependency-light. It is a bridge from the pixel-correspondence
    baseline toward ORB/LK plus ICP, not a claim of full feature-SLAM capability.
    """

    def __init__(self, profile: CameraProfile, *, min_features: int = 40,
                 max_features: int = 400, patch_radius: int = 2,
                 search_radius: int = 5, min_observability: float = 0.02,
                 ransac_iterations: int = 64, ransac_threshold_m: float = 0.04,
                 match_max_score: float = 2.0, match_ratio: float = 0.85,
                 bidirectional_tolerance_px: float = 1.5):
        self.profile = profile
        self.min_features = min_features
        self.max_features = max_features
        self.patch_radius = patch_radius
        self.search_radius = search_radius
        self.min_observability = min_observability
        self.ransac_iterations = ransac_iterations
        self.ransac_threshold_m = ransac_threshold_m
        self.match_max_score = match_max_score
        self.match_ratio = match_ratio
        self.bidirectional_tolerance_px = bidirectional_tolerance_px
        self.previous: RGBDFrame | None = None
        self.pose = np.eye(4)

    def reset(self) -> None:
        self.previous = None
        self.pose = np.eye(4)

    def _features(self, image: np.ndarray, valid: np.ndarray) -> np.ndarray:
        radius = self.patch_radius + self.search_radius
        if image.shape[0] <= 2 * radius or image.shape[1] <= 2 * radius:
            return np.empty((0, 2), dtype=int)
        gx = np.zeros_like(image)
        gy = np.zeros_like(image)
        gx[:, 1:-1] = image[:, 2:] - image[:, :-2]
        gy[1:-1, :] = image[2:, :] - image[:-2, :]
        score = gx * gx + gy * gy
        score[:radius] = 0
        score[-radius:] = 0
        score[:, :radius] = 0
        score[:, -radius:] = 0
        score[~valid] = 0
        candidates = np.argwhere(score > np.percentile(score[score > 0], 70) if np.any(score > 0) else 1.0)
        if len(candidates) == 0:
            return np.empty((0, 2), dtype=int)
        order = candidates[np.argsort(score[candidates[:, 0], candidates[:, 1]])[::-1]]
        selected: list[tuple[int, int]] = []
        cell_h = max(1, image.shape[0] // 12)
        cell_w = max(1, image.shape[1] // 16)
        occupied: set[tuple[int, int]] = set()
        for y, x in order:
            cell = (int(y // cell_h), int(x // cell_w))
            if cell in occupied:
                continue
            selected.append((int(x), int(y)))
            occupied.add(cell)
            if len(selected) >= self.max_features:
                break
        return np.asarray(selected, dtype=int)

    def _match(self, previous: np.ndarray, current: np.ndarray,
               pixels: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        r = self.patch_radius
        s = self.search_radius
        source: list[tuple[int, int]] = []
        target: list[tuple[int, int]] = []
        confidences: list[float] = []
        for x, y in pixels:
            patch = previous[y-r:y+r+1, x-r:x+r+1]
            scores: list[tuple[float, int, int]] = []
            for dy in range(-s, s + 1):
                for dx in range(-s, s + 1):
                    xx, yy = x + dx, y + dy
                    candidate = current[yy-r:yy+r+1, xx-r:xx+r+1]
                    a = patch - patch.mean()
                    b = candidate - candidate.mean()
                    denom = np.linalg.norm(a) * np.linalg.norm(b)
                    score = float(np.sum((a - b) ** 2) / denom) if denom > 1e-9 else float("inf")
                    scores.append((score, xx, yy))
            scores.sort(key=lambda item: item[0])
            best_score, xx, yy = scores[0]
            second_score = scores[1][0] if len(scores) > 1 else float("inf")
            ratio = best_score / max(second_score, 1e-12)
            if best_score <= self.match_max_score and ratio <= self.match_ratio:
                source.append((int(x), int(y)))
                target.append((int(xx), int(yy)))
                confidences.append(float(ratio))
        return (np.asarray(source, dtype=int), np.asarray(target, dtype=int),
                np.asarray(confidences, dtype=float))

    def update(self, frame: RGBDFrame) -> VisualMeasurement:
        if not self.profile.registration_verified:
            return VisualMeasurement(VisualState.DEGRADED, reason="rgb_depth_registration_unverified")
        if self.previous is None:
            self.previous = frame
            return VisualMeasurement(VisualState.TRACKING, transform=self.pose.copy(), reason="initialized")
        if frame.frame_id != self.previous.frame_id:
            return VisualMeasurement(VisualState.DEGRADED, reason="camera_frame_changed")
        dt = frame.stamp - self.previous.stamp
        if not np.isfinite(dt) or dt <= 0.0 or dt > 0.25:
            return VisualMeasurement(VisualState.DEGRADED, reason="stale_or_nonmonotonic_timestamp")
        old_gray, new_gray = _gray(self.previous.color), _gray(frame.color)
        old_valid = _depth_valid_mask(self.previous.depth, self.profile)
        new_valid = _depth_valid_mask(frame.depth, self.profile)
        pixels = self._features(old_gray, old_valid)
        if len(pixels):
            source_px, target_px, match_ratios = self._match(old_gray, new_gray, pixels)
        else:
            source_px = target_px = np.empty((0, 2), dtype=int)
            match_ratios = np.empty(0, dtype=float)
        if len(source_px):
            keep = old_valid[source_px[:, 1], source_px[:, 0]] & new_valid[target_px[:, 1], target_px[:, 0]]
            source_px, target_px, match_ratios = (source_px[keep], target_px[keep], match_ratios[keep])
        if len(source_px) < self.min_features:
            self.previous = frame
            return VisualMeasurement(VisualState.LOST, inliers=len(source_px), reason="insufficient_rgbd_features")
        source = _pixel_points(self.previous.depth, source_px, self.profile)
        target = _pixel_points(frame.depth, target_px, self.profile)
        rotation, translation, inlier_mask = _ransac_rigid(
            source, target, iterations=self.ransac_iterations,
            threshold_m=self.ransac_threshold_m)
        source_inliers, target_inliers = source[inlier_mask], target[inlier_mask]
        rotation, translation, singular, residual = _robust_kabsch(
            source_inliers, target_inliers, trim_fraction=1.0)
        rmse = float(np.sqrt(np.mean(residual * residual)))
        observability = float(singular[-1] / singular[0]) if singular[0] > 0 else 0.0
        rotation_rad = float(np.arccos(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)))
        inlier_count = int(inlier_mask.sum())
        accepted = (inlier_count >= self.min_features and rmse <= 0.05
                    and np.linalg.norm(translation) <= 0.30
                    and rotation_rad <= 0.45 and observability >= self.min_observability)
        if accepted:
            step = np.eye(4)
            step[:3, :3], step[:3, 3] = rotation, translation
            self.pose = self.pose @ np.linalg.inv(step)
            state, reason = VisualState.TRACKING, "feature_rgbd_accepted"
        else:
            state, reason = (VisualState.DEGRADED, "feature_rgbd_quality_gate_rejected"
                             if inlier_count >= self.min_features else "insufficient_geometric_inliers")
        self.previous = frame
        valid_ratio = float(np.count_nonzero(old_valid)) / old_valid.size
        return VisualMeasurement(state, self.pose.copy(), inlier_count, rmse, valid_ratio,
                                 float(np.linalg.norm(translation)), rotation_rad,
                                 observability, reason)
