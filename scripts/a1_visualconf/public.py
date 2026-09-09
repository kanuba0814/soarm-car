"""Public RGB-D geometry and association primitives for A1 VisualConf."""
import hashlib
import math
from collections import defaultdict, deque
from pathlib import Path

import numpy as np

FLOOR_Z_M = 0.0
FLOOR_CLEARANCE_M = 0.05
COMPONENT_DEPTH_ABS_M = 0.05
COMPONENT_DEPTH_REL = 0.02
MIN_COMPONENT_PIXELS = 100
MIN_COMPONENT_RATIO = 0.05


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def quaternion_matrix(quaternion):
    x, y, z, w = (float(quaternion[key]) for key in ("x", "y", "z", "w"))
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0:
        raise ValueError("zero self-pose quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)],
        [2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)],
        [2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)],
    ])


def camera_transform(self_pose, mounting):
    entity = self_pose["entity"]
    body_to_world = quaternion_matrix(entity["orientation"]).T
    body_origin = np.asarray([entity["position"][axis] for axis in ("x", "y", "z")], dtype=float)
    translation = np.asarray(mounting["base_to_camera_translation_m"], dtype=float)
    optical_to_base = np.asarray(mounting["optical_to_base_rows"], dtype=float)
    return body_origin + body_to_world @ translation, body_to_world @ optical_to_base.T


def clip_box(box, width, height):
    left, top, right, bottom = box
    return [max(0, min(width, int(math.floor(left)))), max(0, min(height, int(math.floor(top)))),
            max(0, min(width, int(math.ceil(right)))), max(0, min(height, int(math.ceil(bottom))))]


def box_iou(first, second):
    left, top = max(first[0], second[0]), max(first[1], second[1])
    right, bottom = min(first[2], second[2]), min(first[3], second[3])
    overlap = max(0, right - left) * max(0, bottom - top)
    union = (first[2] - first[0]) * (first[3] - first[1]) + (second[2] - second[0]) * (second[3] - second[1]) - overlap
    return overlap / union if union else 0.0


def backproject(depth, info, pixels):
    fx, fy, cx, cy = info["k"][0], info["k"][4], info["k"][2], info["k"][5]
    rows, columns = pixels[:, 0], pixels[:, 1]
    z = depth[rows, columns].astype(float)
    return np.column_stack(((columns - cx) * z / fx, (rows - cy) * z / fy, z))


def connected_components(valid, depth):
    height, width = valid.shape
    visited = np.zeros_like(valid, dtype=bool)
    components = []
    for row, column in np.argwhere(valid):
        if visited[row, column]:
            continue
        visited[row, column] = True
        pending, points = deque([(int(row), int(column))]), []
        while pending:
            current_row, current_column = pending.popleft()
            points.append((current_row, current_column))
            current_depth = float(depth[current_row, current_column])
            for next_row, next_column in ((current_row - 1, current_column), (current_row + 1, current_column), (current_row, current_column - 1), (current_row, current_column + 1)):
                if next_row < 0 or next_row >= height or next_column < 0 or next_column >= width or not valid[next_row, next_column] or visited[next_row, next_column]:
                    continue
                next_depth = float(depth[next_row, next_column])
                if abs(current_depth - next_depth) <= max(COMPONENT_DEPTH_ABS_M, COMPONENT_DEPTH_REL * max(current_depth, next_depth)):
                    visited[next_row, next_column] = True
                    pending.append((next_row, next_column))
        components.append(np.asarray(points, dtype=int))
    return components


def foreground(depth, invalid, info, transform, box, depth_offset_m=0.0):
    height, width = depth.shape
    left, top, right, bottom = clip_box(box, width, height)
    if right <= left or bottom <= top:
        return {"failure": "empty_clipped_box", "box_xyxy": [left, top, right, bottom]}
    local_depth = depth[top:bottom, left:right].astype(float) + depth_offset_m
    valid = np.isfinite(local_depth) & (local_depth > 0) & (~invalid[top:bottom, left:right])
    valid_count = int(valid.sum())
    if valid_count == 0:
        return {"failure": "foreground_no_valid_depth", "box_xyxy": [left, top, right, bottom], "valid_pixels": 0}
    local_pixels = np.argwhere(valid)
    global_pixels = local_pixels + np.asarray([top, left])
    origin, rotation = transform
    world = backproject(depth + depth_offset_m, info, global_pixels) @ rotation.T + origin
    floor_ok = np.abs(world[:, 2] - FLOOR_Z_M) > FLOOR_CLEARANCE_M
    retained = np.zeros_like(valid)
    floor_pixels = local_pixels[floor_ok]
    retained[floor_pixels[:, 0], floor_pixels[:, 1]] = True
    components = connected_components(retained, local_depth)
    central = (int((right - left) * .2), int((right - left) * .8), int((bottom - top) * .2), int((bottom - top) * .8))
    evidence = []
    for component in components:
        count = len(component)
        support = int(((component[:, 1] >= central[0]) & (component[:, 1] < central[1]) & (component[:, 0] >= central[2]) & (component[:, 0] < central[3])).sum())
        if count >= MIN_COMPONENT_PIXELS and count / valid_count >= MIN_COMPONENT_RATIO:
            evidence.append((support, component))
    component_rows = [{"pixels": int(len(component)), "central_support_pixels": support, "central_support_ratio": support / max(1, len(component))} for support, component in evidence]
    if not evidence:
        return {"failure": "foreground_no_retained_component", "box_xyxy": [left, top, right, bottom], "valid_pixels": valid_count, "floor_removed_pixels": int((~floor_ok).sum()), "components": component_rows}
    evidence.sort(key=lambda item: item[0], reverse=True)
    support, selected = evidence[0]
    runner_support = evidence[1][0] if len(evidence) > 1 else 0
    if support < 2 * runner_support:
        return {"failure": "foreground_ambiguous", "box_xyxy": [left, top, right, bottom], "valid_pixels": valid_count, "floor_removed_pixels": int((~floor_ok).sum()), "components": component_rows}
    pixels = selected + np.asarray([top, left])
    selected_world = backproject(depth + depth_offset_m, info, pixels) @ rotation.T + origin
    return {"failure": None, "box_xyxy": [left, top, right, bottom], "valid_pixels": valid_count, "floor_removed_pixels": int((~floor_ok).sum()), "components": component_rows, "selected_component": {"pixels": int(len(selected)), "central_support_pixels": support, "central_support_ratio": support / max(1, len(selected))}, "pixels_rc": pixels.tolist(), "world_points": selected_world.tolist(), "xy_median_m": np.median(selected_world[:, :2], axis=0).tolist(), "xy_extent_10_90_m": np.percentile(selected_world[:, :2], [10, 90], axis=0).tolist(), "depth_median_m": float(np.median(depth[pixels[:, 0], pixels[:, 1]] + depth_offset_m))}


def add_sensitivity(depth, invalid, info, transform, box, nominal):
    left, top, right, bottom = box
    width, height = right - left, bottom - top
    boxes = {"nominal": box, "inset_5pct": [left + .05 * width, top + .05 * height, right - .05 * width, bottom - .05 * height], "outset_5pct": [left - .05 * width, top - .05 * height, right + .05 * width, bottom + .05 * height]}
    measurements = []
    for name, changed in boxes.items():
        for offset in (-.02, .02):
            result = foreground(depth, invalid, info, transform, changed, offset)
            measurements.append({"box_perturbation": name, "depth_offset_m": offset, "failure": result["failure"], "xy_median_m": result.get("xy_median_m"), "selected_pixels": result.get("selected_component", {}).get("pixels", 0)})
    positions = [item["xy_median_m"] for item in measurements if item["xy_median_m"] is not None]
    origin = np.asarray(nominal["xy_median_m"]) if nominal.get("xy_median_m") else None
    nominal["sensitivity"] = {"measurements": measurements, "max_xy_shift_m": max((float(np.linalg.norm(np.asarray(point) - origin)) for point in positions), default=None) if origin is not None else None, "unstable": len(positions) != len(measurements)}


def project(world_points, transform, info):
    origin, rotation = transform
    camera = (np.asarray(world_points) - origin) @ rotation
    positive = camera[:, 2] > 0
    fx, fy, cx, cy = info["k"][0], info["k"][4], info["k"][2], info["k"][5]
    pixels = np.column_stack((camera[:, 0] * fx / np.maximum(camera[:, 2], 1e-9) + cx, camera[:, 1] * fy / np.maximum(camera[:, 2], 1e-9) + cy))
    return pixels, camera[:, 2], positive


def compatibility(candidate, prior, frames):
    distance = float(np.linalg.norm(np.asarray(candidate["foreground"]["xy_median_m"]) - np.asarray(prior["foreground"]["xy_median_m"])))
    if distance > .30:
        return None
    frame = frames[prior["observation_id"]]
    pixels, z, positive = project(candidate["foreground"]["world_points"], frame["transform"], frame["info"])
    box = prior["box_xyxy"]
    inside = positive & (pixels[:, 0] >= box[0]) & (pixels[:, 0] <= box[2]) & (pixels[:, 1] >= box[1]) & (pixels[:, 1] <= box[3])
    reprojection = float(inside.mean()) if len(inside) else 0.0
    if reprojection < .50:
        return None
    rounded = np.rint(pixels[inside]).astype(int)
    height, width = frame["depth"].shape
    in_image = (rounded[:, 0] >= 0) & (rounded[:, 0] < width) & (rounded[:, 1] >= 0) & (rounded[:, 1] < height)
    if not in_image.any():
        return None
    target_depth = frame["depth"][rounded[in_image, 1], rounded[in_image, 0]]
    valid = ~frame["invalid"][rounded[in_image, 1], rounded[in_image, 0]] & np.isfinite(target_depth) & (target_depth > 0)
    depth_ratio = float((np.abs(z[inside][in_image][valid] - target_depth[valid]) <= .10).mean()) if valid.any() else 0.0
    if depth_ratio < .50:
        return None
    return {"xy_distance_m": distance, "reprojection_ratio": reprojection, "depth_compatibility_ratio": depth_ratio, "cost": distance / .30 + (1 - reprojection) + (1 - depth_ratio)}


def component_overlap(first, second):
    first_pixels, second_pixels = {tuple(point) for point in first["foreground"].get("pixels_rc", [])}, {tuple(point) for point in second["foreground"].get("pixels_rc", [])}
    return len(first_pixels & second_pixels) / len(first_pixels | second_pixels) if first_pixels | second_pixels else 0.0


def associate(detections, frames):
    tracks, audit = [], {"same_frame_duplicates": [], "association_ambiguities": [], "split_ambiguities": []}
    for detection in sorted(detections, key=lambda item: (item["observation_id"], item["detection_index"])):
        if detection["foreground"]["failure"]:
            tracks.append({"track_id": f"track_{len(tracks) + 1:03d}", "label": detection["label"], "detections": [detection], "status": "foreground_unresolved"})
            continue
        options = []
        for track in tracks:
            if track["label"] != detection["label"] or track["status"] != "active":
                continue
            same = [prior for prior in track["detections"] if prior["observation_id"] == detection["observation_id"]]
            if same and not all(box_iou(detection["box_xyxy"], prior["box_xyxy"]) >= .70 and component_overlap(detection, prior) >= .80 for prior in same):
                continue
            checks = [compatibility(detection, prior, frames) for prior in track["detections"]]
            if all(check is not None for check in checks):
                options.append((float(np.mean([check["cost"] for check in checks])), track, bool(same), checks))
        options.sort(key=lambda item: item[0])
        if len(options) > 1 and options[1][0] - options[0][0] < .20:
            audit["association_ambiguities"].append({"detection": [detection["observation_id"], detection["detection_index"]], "track_ids": [options[0][1]["track_id"], options[1][1]["track_id"]], "cost_margin": options[1][0] - options[0][0]})
            tracks.append({"track_id": f"track_{len(tracks) + 1:03d}", "label": detection["label"], "detections": [detection], "status": "association_ambiguous"})
        elif options:
            cost, track, same_frame, checks = options[0]
            detection["association"] = {"selected_track_id": track["track_id"], "cost": cost, "next_alternative_cost_margin": options[1][0] - cost if len(options) > 1 else None, "pair_gates": checks}
            track["detections"].append(detection)
            if same_frame:
                audit["same_frame_duplicates"].append({"track_id": track["track_id"], "detection": [detection["observation_id"], detection["detection_index"]], "component_overlap": component_overlap(detection, track["detections"][-2])})
        else:
            tracks.append({"track_id": f"track_{len(tracks) + 1:03d}", "label": detection["label"], "detections": [detection], "status": "active"})
    by_class = defaultdict(list)
    for track in tracks:
        by_class[track["label"]].append(track)
    for label, class_tracks in by_class.items():
        for index, first in enumerate(class_tracks):
            for second in class_tracks[index + 1:]:
                if first["status"] == second["status"] == "active":
                    first_xy = np.median([item["foreground"]["xy_median_m"] for item in first["detections"]], axis=0)
                    second_xy = np.median([item["foreground"]["xy_median_m"] for item in second["detections"]], axis=0)
                    if np.linalg.norm(first_xy - second_xy) <= .30:
                        audit["split_ambiguities"].append({"label": label, "track_ids": [first["track_id"], second["track_id"]], "reason": "nearby_tracks_preserved_without_transitive_merge"})
    return tracks, audit


def voxelize(detections):
    arrays = [np.asarray(item["foreground"]["world_points"], dtype=float) for item in detections if item["foreground"].get("world_points")]
    if not arrays:
        return []
    points, grouped = np.vstack(arrays), {}
    for key, point in zip(map(tuple, np.floor(points / .02).astype(int)), points):
        grouped.setdefault(key, []).append(point)
    return [np.mean(value, axis=0).tolist() for value in grouped.values()]


def score_tracks(tracks):
    for track in tracks:
        usable = [item for item in track["detections"] if not item["foreground"]["failure"]]
        positions = np.asarray([item["foreground"]["xy_median_m"] for item in usable], dtype=float)
        track["V"] = max(item["confidence"] for item in track["detections"])
        distinct = len({item["observation_id"] for item in usable})
        residual = float(np.median(np.linalg.norm(positions - np.median(positions, axis=0), axis=1))) if len(positions) > 1 else 0.0
        track["M"] = min(1.0, (distinct - 1) / 2) * math.exp(-residual / .10) if distinct > 1 else 0.0
        track["M_evidence"] = {"distinct_observations": distinct, "median_xy_residual_m": residual}
        track["surface_voxels_2cm"] = voxelize(usable)


def surface_gap(first, second):
    one, two = np.asarray(first["surface_voxels_2cm"]), np.asarray(second["surface_voxels_2cm"])
    if not len(one) or not len(two):
        return None
    distances = np.linalg.norm(one[:, None, :2] - two[None, :, :2], axis=2)
    return float(max(np.percentile(distances.min(axis=1), 10), np.percentile(distances.min(axis=0), 10)))


def relation_values(kind, tracks, heading):
    chairs = [track for track in tracks if track["label"] == "chair" and track["status"] == "active"]
    anchors = [track for track in tracks if track["label"] == "dining table" and track["status"] == "active"]
    if len(anchors) != 1:
        return None, "missing_or_ambiguous_anchor"
    if not chairs:
        return None, "no_public_candidate"
    anchor = anchors[0]
    if not anchor["surface_voxels_2cm"] or any(not chair["surface_voxels_2cm"] for chair in chairs):
        return None, "incomplete_observed_surface"
    anchor_xy = np.median(np.asarray(anchor["surface_voxels_2cm"])[:, :2], axis=0)
    lateral = np.array([-math.sin(heading), math.cos(heading)])
    values = {}
    for chair in chairs:
        if kind in ("nearest_chair_table", "farthest_chair_table"):
            values[chair["track_id"]] = surface_gap(chair, anchor)
        else:
            values[chair["track_id"]] = float(np.dot(np.median(np.asarray(chair["surface_voxels_2cm"])[:, :2], axis=0) - anchor_xy, lateral))
    return values, None


def make_records(config, tracks, reference_heading):
    records = []
    for variant in ("V", "V+R", "V+M", "V+R+M"):
        for query in config["queries"]:
            kind = query["kind"]
            label = "couch" if kind == "couch" else "chair"
            candidates = [track for track in tracks if track["label"] == label and track["status"] == "active"]
            relational = kind not in ("chair", "couch")
            relation, relation_reason = relation_values(kind, tracks, reference_heading) if relational else ({}, None)
            ranking = []
            for track in candidates:
                r_value = None
                if relational and relation is not None:
                    values = list(relation.values())
                    r_value = (relation[track["track_id"]] - min(values)) / max(1e-9, max(values) - min(values))
                    if kind.startswith(("leftmost", "nearest")):
                        r_value = 1.0 - r_value
                enabled = [track["V"]] + ([track["M"]] if "+M" in variant else []) + ([r_value] if "+R" in variant and relational and r_value is not None else [])
                ranking.append({"track_id": track["track_id"], "score": float(sum(enabled) / len(enabled)), "V": track["V"], "M": track["M"], "R": r_value})
            ranking.sort(key=lambda item: item["score"], reverse=True)
            uncertainty = max((item["foreground"].get("sensitivity", {}).get("max_xy_shift_m") or 0 for track in candidates for item in track["detections"]), default=0)
            accepted, reason = None, None
            if not ranking:
                reason = "no_public_candidate"
            elif relational and "+R" in variant and relation is None:
                reason = relation_reason
            elif relational and "+R" in variant and len(relation) > 1 and abs(sorted(relation.values())[-1] - sorted(relation.values())[-2]) <= .15 + 2 * uncertainty:
                reason = "relation_ambiguity_margin"
            elif len(ranking) > 1 and ranking[0]["score"] - ranking[1]["score"] < .05:
                reason = "top_two_margin"
            else:
                accepted = ranking[0]["track_id"]
            records.append({"variant": variant, "query_id": query["id"], "query_kind": kind, "forced_ranking": ranking, "accepted_track_id": accepted, "abstain_reason": reason, "relation_status": "unknown" if relational and relation is None else "measured_or_not_required", "relation_reason": relation_reason, "relation_values": relation, "uncertainty_margin_m": .15 + 2 * uncertainty if relational else None})
    return records
