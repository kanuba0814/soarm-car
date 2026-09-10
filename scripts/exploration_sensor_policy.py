"""Pure, replayable source policy for prior-map exploration observations."""
import math

import numpy as np

# Frozen kuruma_ros2_control electronics_envelope visual geometry in base_link.
SELF_MODEL_SHA256 = "a8d8f3c3ad114ebd7ac99408de5e3d382c06342ca98a4a13eea8658e404307eb"
SELF_BOXES = (
    (-0.18, 0.18, -0.18, 0.18, -0.03, 0.03),  # electronics_collision
    (-0.15, 0.15, -0.13, 0.13, 0.02, 0.12),  # electronics_envelope
    (-0.30, 0.30, 0.27, 0.30, -0.015, 0.015),
    (-0.30, 0.30, -0.30, -0.27, -0.015, 0.015),
    (0.27, 0.30, -0.27, 0.27, -0.015, 0.015),
    (-0.30, -0.27, -0.27, 0.27, -0.015, 0.015),
)
SELF_SPHERES = tuple((x, y, 0.0, .0763) for x in (-.1736, .1736) for y in (-.2228, .2228))


def rotation_matrix(quaternion):
    x, y, z, w = quaternion
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def transform_points(points, translation, quaternion):
    values = np.asarray(points, dtype=float)
    return values @ rotation_matrix(quaternion).T + np.asarray(translation, dtype=float)


def self_mask(base_points):
    points = np.asarray(base_points, dtype=float)
    masked = np.zeros(len(points), dtype=bool)
    for minimum_x, maximum_x, minimum_y, maximum_y, minimum_z, maximum_z in SELF_BOXES:
        tolerance = 1e-9
        masked |= ((minimum_x - tolerance <= points[:, 0]) & (points[:, 0] <= maximum_x + tolerance) &
                   (minimum_y - tolerance <= points[:, 1]) & (points[:, 1] <= maximum_y + tolerance) &
                   (minimum_z - tolerance <= points[:, 2]) & (points[:, 2] <= maximum_z + tolerance))
    for center_x, center_y, center_z, radius in SELF_SPHERES:
        masked |= ((points[:, 0] - center_x) ** 2 + (points[:, 1] - center_y) ** 2 +
                   (points[:, 2] - center_z) ** 2 <= radius ** 2 + 1e-9)
    return masked


def select_rays(raw_points, base_points, map_points, ground_reference_z, *, max_rays=1024,
                azimuth_bins=720, floor_tolerance=.03, navigation_z_min=.03,
                navigation_z_max=1.50):
    """Retain the nearest non-self navigation return per azimuth, then visibility extras."""
    raw = np.asarray(raw_points, dtype=float)
    base = np.asarray(base_points, dtype=float)
    mapped = np.asarray(map_points, dtype=float)
    finite = np.isfinite(raw).all(axis=1) & np.isfinite(base).all(axis=1) & np.isfinite(mapped).all(axis=1)
    own = self_mask(base) & finite
    usable = finite & ~own
    relative_height = mapped[:, 2] - ground_reference_z
    navigation = usable & (relative_height >= navigation_z_min) & (relative_height <= navigation_z_max)
    indices = np.flatnonzero(navigation)
    if len(indices):
        azimuth = np.arctan2(base[indices, 1], base[indices, 0])
        bins = np.minimum(azimuth_bins - 1, np.maximum(0, ((azimuth + math.pi) * azimuth_bins / (2 * math.pi)).astype(int)))
        ranges = np.linalg.norm(base[indices], axis=1)
        ordered = indices[np.lexsort((ranges, bins))]
        ordered_bins = bins[np.lexsort((ranges, bins))]
        selected_navigation = ordered[np.r_[True, ordered_bins[1:] != ordered_bins[:-1]]]
    else:
        selected_navigation = np.empty(0, dtype=int)
    selected = list(selected_navigation[:max_rays])
    visibility = np.flatnonzero(usable & ~navigation)
    remaining = max_rays - len(selected)
    if remaining and len(visibility):
        positions = np.linspace(0, len(visibility) - 1, min(remaining, len(visibility)), dtype=int)
        selected.extend(visibility[positions].tolist())
    selected = np.asarray(selected, dtype=int)
    policy = {
        'name': 'nearest_nonself_navigation_per_azimuth_then_visibility_extras',
        'azimuth_bins': azimuth_bins,
        'raw_count': int(len(raw)),
        'finite_count': int(finite.sum()),
        'self_filtered_count': int(own.sum()),
        'navigation_candidates': int(navigation.sum()),
        'navigation_bins_retained': int(len(selected_navigation)),
        'visibility_extras_retained': int(max(0, len(selected) - len(selected_navigation))),
        'selected_count': int(len(selected)),
        'ground_reference_z_map': float(ground_reference_z),
        'floor_tolerance_m': floor_tolerance,
        'navigation_height_band_relative_base_footprint_m': [navigation_z_min, navigation_z_max],
        'self_model_sha256': SELF_MODEL_SHA256,
        'self_geometry': {'boxes_base_link': [list(box) for box in SELF_BOXES], 'spheres_base_link': [list(sphere) for sphere in SELF_SPHERES]},
    }
    return mapped[selected], policy
