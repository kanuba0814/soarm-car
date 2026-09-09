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
REQUIRED_RULES = {
    "tilt_limit_deg", "detector_confidence", "floor_z_m", "floor_clearance_m",
    "track_xy_gate_m", "viewgroup_camera_center_m", "viewgroup_bearing_deg", "reprojection_gate", "depth_compatibility_m",
    "association_cost_margin", "duplicate_iou", "duplicate_component_overlap",
    "component_depth_abs_m", "component_depth_relative", "component_min_pixels",
    "component_min_valid_ratio", "central_support_ratio", "box_perturbation_fraction",
    "depth_perturbations_m", "top_two_margin", "relation_ambiguity_m",
    "relation_side_margin_m", "surface_voxel_m",
}


def validate_rules(rules):
    missing = REQUIRED_RULES - set(rules)
    unknown = set(rules) - REQUIRED_RULES
    if missing or unknown:
        raise ValueError(f"invalid A1 rules: missing={sorted(missing)} unknown={sorted(unknown)}")
    if len(rules["depth_perturbations_m"]) != 2 or rules["track_xy_gate_m"] <= 0 or rules["surface_voxel_m"] <= 0:
        raise ValueError("invalid A1 rule values")
    return rules


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


def connected_components_with_rules(valid, depth, rules):
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
                if abs(current_depth - next_depth) <= max(rules["component_depth_abs_m"], rules["component_depth_relative"] * max(current_depth, next_depth)):
                    visited[next_row, next_column] = True
                    pending.append((next_row, next_column))
        components.append(np.asarray(points, dtype=int))
    return components


def voxelize_points(points, size):
    points = np.asarray(points, dtype=float)
    if not len(points):
        return []
    grouped = {}
    for key, point in zip(map(tuple, np.floor(points / size).astype(int)), points):
        grouped.setdefault(key, []).append(point)
    return [np.mean(group, axis=0).tolist() for group in grouped.values()]


def foreground(depth, invalid, info, transform, box, rules, depth_offset_m=0.0):
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
    floor_ok = np.abs(world[:, 2] - rules["floor_z_m"]) > rules["floor_clearance_m"]
    retained = np.zeros_like(valid)
    floor_pixels = local_pixels[floor_ok]
    retained[floor_pixels[:, 0], floor_pixels[:, 1]] = True
    components = connected_components_with_rules(retained, local_depth, rules)
    central = (int((right - left) * .2), int((right - left) * .8), int((bottom - top) * .2), int((bottom - top) * .8))
    evidence = []
    for component in components:
        count = len(component)
        support = int(((component[:, 1] >= central[0]) & (component[:, 1] < central[1]) & (component[:, 0] >= central[2]) & (component[:, 0] < central[3])).sum())
        if count >= rules["component_min_pixels"] and count / valid_count >= rules["component_min_valid_ratio"]:
            evidence.append((support, component))
    component_rows = [{"pixels": int(len(component)), "central_support_pixels": support, "central_support_ratio": support / max(1, len(component))} for support, component in evidence]
    if not evidence:
        return {"failure": "foreground_no_retained_component", "box_xyxy": [left, top, right, bottom], "valid_pixels": valid_count, "floor_removed_pixels": int((~floor_ok).sum()), "components": component_rows}
    evidence.sort(key=lambda item: item[0], reverse=True)
    support, selected = evidence[0]
    runner_support = evidence[1][0] if len(evidence) > 1 else 0
    if support < rules["central_support_ratio"] * runner_support:
        return {"failure": "foreground_ambiguous", "box_xyxy": [left, top, right, bottom], "valid_pixels": valid_count, "floor_removed_pixels": int((~floor_ok).sum()), "components": component_rows}
    pixels = selected + np.asarray([top, left])
    selected_world = backproject(depth + depth_offset_m, info, pixels) @ rotation.T + origin
    return {"failure": None, "box_xyxy": [left, top, right, bottom], "valid_pixels": valid_count, "floor_removed_pixels": int((~floor_ok).sum()), "components": component_rows, "selected_component": {"pixels": int(len(selected)), "central_support_pixels": support, "central_support_ratio": support / max(1, len(selected))}, "pixels_rc": pixels.tolist(), "world_points": selected_world.tolist(), "xy_median_m": np.median(selected_world[:, :2], axis=0).tolist(), "xy_extent_10_90_m": np.percentile(selected_world[:, :2], [10, 90], axis=0).tolist(), "depth_median_m": float(np.median(depth[pixels[:, 0], pixels[:, 1]] + depth_offset_m))}


def add_sensitivity(depth, invalid, info, transform, box, nominal, rules):
    left, top, right, bottom = box
    width, height = right - left, bottom - top
    fraction = rules["box_perturbation_fraction"]
    boxes = {"nominal": box, "inset": [left + fraction * width, top + fraction * height, right - fraction * width, bottom - fraction * height], "outset": [left - fraction * width, top - fraction * height, right + fraction * width, bottom + fraction * height]}
    measurements = []
    for name, changed in boxes.items():
        for offset in rules["depth_perturbations_m"]:
            result = foreground(depth, invalid, info, transform, changed, rules, offset)
            measurements.append({"box_perturbation": name, "depth_offset_m": offset, "failure": result["failure"], "xy_median_m": result.get("xy_median_m"), "selected_pixels": result.get("selected_component", {}).get("pixels", 0), "surface_voxels": voxelize_points(result.get("world_points", []), rules["surface_voxel_m"])})
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


def compatibility(candidate, prior, frames, rules):
    distance = float(np.linalg.norm(np.asarray(candidate["foreground"]["xy_median_m"]) - np.asarray(prior["foreground"]["xy_median_m"])))
    if distance > rules["track_xy_gate_m"]:
        return None
    frame = frames[prior["observation_id"]]
    pixels, z, positive = project(candidate["foreground"]["world_points"], frame["transform"], frame["info"])
    box = prior["box_xyxy"]
    inside = positive & (pixels[:, 0] >= box[0]) & (pixels[:, 0] <= box[2]) & (pixels[:, 1] >= box[1]) & (pixels[:, 1] <= box[3])
    reprojection = float(inside.mean()) if len(inside) else 0.0
    if reprojection < rules["reprojection_gate"]:
        return None
    rounded = np.rint(pixels[inside]).astype(int)
    height, width = frame["depth"].shape
    in_image = (rounded[:, 0] >= 0) & (rounded[:, 0] < width) & (rounded[:, 1] >= 0) & (rounded[:, 1] < height)
    if not in_image.any():
        return None
    target_depth = frame["depth"][rounded[in_image, 1], rounded[in_image, 0]]
    valid = ~frame["invalid"][rounded[in_image, 1], rounded[in_image, 0]] & np.isfinite(target_depth) & (target_depth > 0)
    depth_ratio = float((np.abs(z[inside][in_image][valid] - target_depth[valid]) <= rules["depth_compatibility_m"]).mean()) if valid.any() else 0.0
    if depth_ratio < rules["reprojection_gate"]:
        return None
    return {"xy_distance_m": distance, "reprojection_ratio": reprojection, "depth_compatibility_ratio": depth_ratio, "cost": distance / rules["track_xy_gate_m"] + (1 - reprojection) + (1 - depth_ratio)}


def component_overlap(first, second):
    first_pixels, second_pixels = {tuple(point) for point in first["foreground"].get("pixels_rc", [])}, {tuple(point) for point in second["foreground"].get("pixels_rc", [])}
    return len(first_pixels & second_pixels) / len(first_pixels | second_pixels) if first_pixels | second_pixels else 0.0


def associate(detections, frames, rules):
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
            if same and not all(box_iou(detection["box_xyxy"], prior["box_xyxy"]) >= rules["duplicate_iou"] and component_overlap(detection, prior) >= rules["duplicate_component_overlap"] for prior in same):
                continue
            checks = [compatibility(detection, prior, frames, rules) for prior in track["detections"]]
            if all(check is not None for check in checks):
                options.append((float(np.mean([check["cost"] for check in checks])), track, bool(same), checks))
        options.sort(key=lambda item: item[0])
        if len(options) > 1 and options[1][0] - options[0][0] < rules["association_cost_margin"]:
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
                    if np.linalg.norm(first_xy - second_xy) <= rules["track_xy_gate_m"]:
                        audit["split_ambiguities"].append({"label": label, "track_ids": [first["track_id"], second["track_id"]], "reason": "nearby_tracks_preserved_without_transitive_merge"})
    return tracks, audit


def voxelize(detections, rules):
    arrays = [np.asarray(item["foreground"]["world_points"], dtype=float) for item in detections if item["foreground"].get("world_points")]
    return voxelize_points(np.vstack(arrays), rules["surface_voxel_m"]) if arrays else []


def view_groups(detections, rules):
    groups = []
    bearing_limit = math.radians(rules["viewgroup_bearing_deg"])
    for detection in sorted(detections, key=lambda item: (item["observation_id"], item["detection_index"])):
        center = np.asarray(detection.get("camera_center_xy_m", [0.0, 0.0]), dtype=float)
        bearing = float(detection.get("camera_bearing_rad", 0.0))
        separate = True
        for group in groups:
            center_delta = np.linalg.norm(center - np.asarray(group["camera_center_xy_m"]))
            bearing_delta = abs((bearing - group["camera_bearing_rad"] + math.pi) % (2 * math.pi) - math.pi)
            if center_delta < rules["viewgroup_camera_center_m"] and bearing_delta < bearing_limit:
                group["detections"].append([detection["observation_id"], detection["detection_index"]])
                separate = False
                break
        if separate:
            groups.append({"group_id": f"viewgroup_{len(groups) + 1:03d}", "camera_center_xy_m": center.tolist(), "camera_bearing_rad": bearing, "detections": [[detection["observation_id"], detection["detection_index"]]]})
    return groups


def score_tracks(tracks, rules):
    for track in tracks:
        usable = [item for item in track["detections"] if not item["foreground"]["failure"]]
        positions = np.asarray([item["foreground"]["xy_median_m"] for item in usable], dtype=float)
        track["V"] = max(item["confidence"] for item in track["detections"])
        groups = view_groups(usable, rules)
        distinct = len(groups)
        residual = float(np.median(np.linalg.norm(positions - np.median(positions, axis=0), axis=1))) if len(positions) > 1 else 0.0
        track["M"] = min(1.0, (distinct - 1) / 2) * math.exp(-residual / rules["depth_compatibility_m"]) if distinct > 1 else 0.0
        track["M_evidence"] = {"decorrelated_viewgroups": distinct, "median_xy_residual_m": residual, "groups": groups}
        track["surface_voxels_2cm"] = voxelize(usable, rules)
        alternatives = [track["surface_voxels_2cm"]]
        failures = []
        for detection in usable:
            sensitivity = detection["foreground"].get("sensitivity", {})
            for measurement in sensitivity.get("measurements", []):
                if measurement["failure"] or not measurement["surface_voxels"]:
                    failures.append({"observation_id": detection["observation_id"], "measurement": measurement})
                else:
                    alternatives.append(measurement["surface_voxels"])
        track["relation_surface_envelopes"] = alternatives
        track["relation_geometry_failures"] = failures


def surface_gap_points(first, second):
    one, two = np.asarray(first), np.asarray(second)
    if not len(one) or not len(two):
        return None
    distances = np.linalg.norm(one[:, None, :2] - two[None, :, :2], axis=2)
    return float(max(np.percentile(distances.min(axis=1), 10), np.percentile(distances.min(axis=0), 10)))


def relation_values(kind, tracks, heading, rules):
    chairs = [track for track in tracks if track["label"] == "chair" and track["status"] == "active"]
    anchors = [track for track in tracks if track["label"] == "dining table" and track["status"] == "active"]
    if len(anchors) != 1:
        return None, "missing_or_ambiguous_anchor"
    if not chairs:
        return None, "no_public_candidate"
    anchor = anchors[0]
    required = [anchor] + chairs
    if any(track.get("relation_geometry_failures") or not track.get("relation_surface_envelopes") for track in required):
        return None, "perturbation_geometry_failed"
    # Frozen horizontal convention: RIGHT is positive, so LEFT is negative.
    lateral = np.array([math.sin(heading), -math.cos(heading)])
    intervals = {}
    for chair in chairs:
        values = []
        for candidate_surface in chair["relation_surface_envelopes"]:
            for anchor_surface in anchor["relation_surface_envelopes"]:
                if kind in ("nearest_chair_table", "farthest_chair_table"):
                    value = surface_gap_points(candidate_surface, anchor_surface)
                else:
                    value = float(np.dot(np.median(np.asarray(candidate_surface)[:, :2], axis=0) - np.median(np.asarray(anchor_surface)[:, :2], axis=0), lateral))
                if value is None:
                    return None, "perturbation_geometry_failed"
                values.append(value)
        intervals[chair["track_id"]] = {"lower": min(values), "upper": max(values), "samples": values}
    return intervals, None


def choose_relation(kind, intervals, rules):
    margin = rules["relation_ambiguity_m"]
    side_margin = rules["relation_side_margin_m"]
    if kind.startswith("leftmost"):
        eligible = {key: value for key, value in intervals.items() if value["upper"] < -side_margin}
        if not eligible:
            return None, "no_definite_left_side"
        winner = min(eligible, key=lambda key: eligible[key]["upper"])
        if any(key != winner and eligible[key]["lower"] <= eligible[winner]["upper"] + margin for key in eligible):
            return None, "relation_order_changes_under_perturbation"
    elif kind.startswith("rightmost"):
        eligible = {key: value for key, value in intervals.items() if value["lower"] > side_margin}
        if not eligible:
            return None, "no_definite_right_side"
        winner = max(eligible, key=lambda key: eligible[key]["lower"])
        if any(key != winner and eligible[key]["upper"] >= eligible[winner]["lower"] - margin for key in eligible):
            return None, "relation_order_changes_under_perturbation"
    elif kind.startswith("nearest"):
        winner = min(intervals, key=lambda key: intervals[key]["upper"])
        if any(key != winner and intervals[key]["lower"] <= intervals[winner]["upper"] + margin for key in intervals):
            return None, "relation_order_changes_under_perturbation"
    else:
        winner = max(intervals, key=lambda key: intervals[key]["lower"])
        if any(key != winner and intervals[key]["upper"] >= intervals[winner]["lower"] - margin for key in intervals):
            return None, "relation_order_changes_under_perturbation"
    return winner, None


def make_records(config, tracks, reference_heading):
    rules = validate_rules(config["rules"])
    records = []
    for variant in ("V", "V+R", "V+M", "V+R+M"):
        for query in config["queries"]:
            kind = query["kind"]
            label = "couch" if kind == "couch" else "chair"
            candidates = [track for track in tracks if track["label"] == label and track["status"] == "active"]
            relational = kind not in ("chair", "couch")
            relation, relation_reason = relation_values(kind, tracks, reference_heading, rules) if relational else ({}, None)
            winner, stability_reason = choose_relation(kind, relation, rules) if relation is not None and relational else (None, relation_reason)
            ranking = []
            for track in candidates:
                r_value = None
                if relational and relation is not None:
                    interval = relation[track["track_id"]]
                    r_value = 1.0 if track["track_id"] == winner else 0.0
                    if winner is None:
                        r_value = None
                enabled = [track["V"]] + ([track["M"]] if "+M" in variant else []) + ([r_value] if "+R" in variant and relational and r_value is not None else [])
                ranking.append({"track_id": track["track_id"], "score": float(sum(enabled) / len(enabled)), "V": track["V"], "M": track["M"], "R": r_value})
            ranking.sort(key=lambda item: item["score"], reverse=True)
            accepted, reason = None, None
            if not ranking:
                reason = "no_public_candidate"
            elif relational and "+R" in variant and (relation is None or winner is None):
                reason = stability_reason
            elif len(ranking) > 1 and ranking[0]["score"] - ranking[1]["score"] < rules["top_two_margin"]:
                reason = "top_two_margin"
            else:
                accepted = ranking[0]["track_id"]
            records.append({"variant": variant, "query_id": query["id"], "query_kind": kind, "forced_ranking": ranking, "accepted_track_id": accepted, "abstain_reason": reason, "relation_status": "unknown" if relational and (relation is None or winner is None) else "measured_or_not_required", "relation_reason": stability_reason, "relation_intervals": relation, "uncertainty_margin_m": rules["relation_ambiguity_m"] if relational else None})
    return records
