#!/usr/bin/env python3
"""Independent private A1 evaluator. It never feeds GT back to the public method."""
import argparse
import hashlib
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np
from PIL import Image

from a1_visualconf.public import backproject, camera_transform, quaternion_matrix, validate_rules


TAXONOMY = {
    "ChairA_01": "chair",
    "ChairD_01": "chair",
    "KitchenTable_01": "dining table",
    "SofaC_01": "couch",
    "CoffeeTable_01": "coffee table",
    "Tablet_01": "tablet",
}
IOU_GATE = .50


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def taxonomy(source_name):
    asset = source_name.rsplit("_", 1)[0]
    return TAXONOMY.get(asset)


def iou(first, second):
    left, top = max(first[0], second[0]), max(first[1], second[1])
    right, bottom = min(first[2], second[2]), min(first[3], second[3])
    overlap = max(0, right - left) * max(0, bottom - top)
    union = (first[2] - first[0]) * (first[3] - first[1]) + (second[2] - second[0]) * (second[3] - second[1]) - overlap
    return overlap / union if union else 0.0


def heading(quaternion):
    rotation = quaternion_matrix(quaternion).T
    forward = rotation @ np.array([1.0, 0.0, 0.0])
    return math.atan2(forward[1], forward[0])


def voxelize(points, size):
    if not len(points):
        return []
    groups = {}
    for key, point in zip(map(tuple, np.floor(points / size).astype(int)), points):
        groups.setdefault(key, []).append(point)
    return [np.mean(group, axis=0).tolist() for group in groups.values()]


def load_ground_truth(private_dir, public_dir, public_input, rules):
    package = json.loads((private_dir / "instances.json").read_text())
    calibration = json.loads((public_input / "calibration_selfpose.json").read_text())["observations"]
    instances = {}
    per_observation = defaultdict(list)
    for observation in package["observations"]:
        observation_id = observation["observation_id"]
        root = public_dir / observation_id
        depth = np.load(root / "depth_m_float32.npy")
        invalid = np.load(root / "depth_invalid_mask.npy")
        info = json.loads((root / "camera_info.json").read_text())
        transform = camera_transform(calibration[observation_id]["self_pose"], calibration[observation_id]["camera_mounting"])
        for visible in observation["visible_instances"]:
            label = taxonomy(visible["source_name"])
            if label not in ("chair", "dining table", "couch"):
                continue
            mask_path = private_dir / "masks" / observation_id / f"{visible['label_id']:03d}.png"
            mask = np.asarray(Image.open(mask_path).convert("L")) > 0
            pixels = np.argwhere(mask & ~invalid & np.isfinite(depth) & (depth > 0))
            points = backproject(depth, info, pixels) @ transform[1].T + transform[0] if len(pixels) else np.empty((0, 3))
            key = visible["source_name"]
            instance = instances.setdefault(key, {"identity": key, "label": label, "observations": [], "points": []})
            instance["observations"].append({"observation_id": observation_id, "label_id": visible["label_id"], "bbox_xyxy": visible["bbox_xyxy"], "mask_pixels": int(mask.sum()), "valid_depth_pixels": int(len(pixels)), "mask_path": str(mask_path)})
            if len(points):
                instance["points"].append(points)
            per_observation[observation_id].append({"identity": key, "label": label, "bbox_xyxy": visible["bbox_xyxy"], "label_id": visible["label_id"], "mask": mask, "valid_depth_pixels": int(len(pixels))})
    for instance in instances.values():
        points = np.vstack(instance.pop("points")) if instance["points"] else np.empty((0, 3))
        instance["surface_voxels_2cm"] = voxelize(points, rules["surface_voxel_m"])
        instance["observed_surface_status"] = "available" if instance["surface_voxels_2cm"] else "insufficient_geometry"
    return instances, per_observation


def max_weight_assignment(predictions, targets):
    """Deterministic exhaustive maximum-IoU assignment for the small frame sets."""
    weights = [[iou(prediction["box_xyxy"], target["bbox_xyxy"]) if prediction["label"] == target["label"] else 0.0 for target in targets] for prediction in predictions]
    best = (-1.0, ())

    def visit(index, used, pairs, total):
        nonlocal best
        if index == len(predictions):
            candidate = (total, tuple(pairs))
            if candidate[0] > best[0] or (candidate[0] == best[0] and candidate[1] < best[1]):
                best = candidate
            return
        visit(index + 1, used, pairs, total)
        for target_index, weight in enumerate(weights[index]):
            if target_index not in used and weight >= IOU_GATE:
                visit(index + 1, used | {target_index}, pairs + [(index, target_index, weight)], total + weight)

    visit(0, set(), [], 0.0)
    return {prediction_index: (targets[target_index], weight) for prediction_index, target_index, weight in best[1]}, weights


def constituent_assignments(method, gt_by_observation):
    assignments, diagnostics = {}, []
    by_observation = defaultdict(list)
    for track in method["tracks"]:
        for detection in track["detections"]:
            by_observation[detection["observation_id"]].append((track["track_id"], detection))
    for observation_id, entries in sorted(by_observation.items()):
        predictions = [{"track_id": track_id, **detection} for track_id, detection in entries]
        matched, weights = max_weight_assignment(predictions, gt_by_observation[observation_id])
        for index, prediction in enumerate(predictions):
            target = matched.get(index)
            key = (prediction["track_id"], prediction["observation_id"], prediction["detection_index"])
            if target is None:
                assignments[key] = None
                continue
            proposed = {tuple(pixel) for pixel in prediction["foreground"].get("pixels_rc", [])}
            observed = {tuple(pixel) for pixel in np.argwhere(target[0]["mask"])}
            overlap = len(proposed & observed)
            assignments[key] = {"identity": target[0]["identity"], "iou": target[1], "gt_label_id": target[0]["label_id"], "foreground": {"status": "measured" if proposed else "no_public_foreground", "proposed_pixels": len(proposed), "private_mask_pixels": len(observed), "overlap_pixels": overlap, "purity": overlap / len(proposed) if proposed else None, "coverage": overlap / len(observed) if observed else None}}
        diagnostics.append({"observation_id": observation_id, "predictions": len(predictions), "targets": len(gt_by_observation[observation_id]), "class_matched_iou_gate": IOU_GATE, "matches": sum(item is not None for item in matched.values()), "weight_matrix": weights})
    return assignments, diagnostics


def track_identities(method, assignments):
    mapped = {}
    for track in method["tracks"]:
        values = []
        unmatched = 0
        for detection in track["detections"]:
            result = assignments[(track["track_id"], detection["observation_id"], detection["detection_index"])]
            if result is None:
                unmatched += 1
            else:
                values.append(result)
        identities = sorted({item["identity"] for item in values})
        if len(identities) > 1:
            status = "contaminated_distinct_identities"
        elif unmatched:
            status = "uncertain_unmatched_constituent"
        elif len(identities) == 1:
            status = "verified"
        else:
            status = "unmatched"
        mapped[track["track_id"]] = {"status": status, "identity": identities[0] if status == "verified" else None, "distinct_identities": identities, "matched_constituents": len(values), "unmatched_constituents": unmatched, "constituents": values}
    return mapped


def surface_gap(first, second):
    one, two = np.asarray(first["surface_voxels_2cm"]), np.asarray(second["surface_voxels_2cm"])
    if not len(one) or not len(two):
        return None
    distances = np.linalg.norm(one[:, None, :2] - two[None, :, :2], axis=2)
    return float(max(np.percentile(distances.min(axis=1), 10), np.percentile(distances.min(axis=0), 10)))


def ground_truth_answers(instances, reference_heading, rules):
    chairs = [item for item in instances.values() if item["label"] == "chair"]
    tables = [item for item in instances.values() if item["label"] == "dining table"]
    couches = [item for item in instances.values() if item["label"] == "couch"]
    answers = {"Q1": {"status": "definite", "identities": sorted(item["identity"] for item in chairs), "reason": None}, "Q2": {"status": "definite" if couches else "insufficient_geometry", "identities": sorted(item["identity"] for item in couches), "reason": None if couches else "no_observed_couch"}}
    if len(tables) != 1 or not chairs or not tables[0]["surface_voxels_2cm"] or any(not item["surface_voxels_2cm"] for item in chairs):
        for query in ("Q3", "Q4", "Q5", "Q6"):
            answers[query] = {"status": "insufficient_geometry", "identities": [], "reason": "missing_anchor_or_observed_surface"}
        return answers
    table = tables[0]
    table_xy = np.median(np.asarray(table["surface_voxels_2cm"])[:, :2], axis=0)
    # Independent evaluator uses the same documented RIGHT-positive axis.
    lateral = np.array([math.sin(reference_heading), -math.cos(reference_heading)])
    uncertainty = 2 * rules["surface_voxel_m"]
    side = {item["identity"]: float(np.dot(np.median(np.asarray(item["surface_voxels_2cm"])[:, :2], axis=0) - table_xy, lateral)) for item in chairs}
    gaps = {item["identity"]: surface_gap(item, table) for item in chairs}
    definitions = {"Q3": (side, min, "left"), "Q4": (side, max, "right"), "Q5": (gaps, min, "nearest"), "Q6": (gaps, max, "farthest")}
    for query, (values, selector, relation) in definitions.items():
        ranked = sorted(values.items(), key=lambda item: item[1], reverse=selector is max)
        if any(value is None for _, value in ranked) or not ranked:
            answers[query] = {"status": "insufficient_geometry", "identities": [], "reason": "surface_gap_unavailable"}
        elif len(ranked) > 1 and abs(ranked[0][1] - ranked[1][1]) <= rules["relation_ambiguity_m"] + uncertainty:
            answers[query] = {"status": "ambiguous", "identities": [], "reason": "relation_ambiguity_margin"}
        elif relation == "left" and ranked[0][1] + uncertainty >= -rules["relation_side_margin_m"]:
            answers[query] = {"status": "ambiguous", "identities": [], "reason": "no_definite_left_surface"}
        elif relation == "right" and ranked[0][1] - uncertainty <= rules["relation_side_margin_m"]:
            answers[query] = {"status": "ambiguous", "identities": [], "reason": "no_definite_right_surface"}
        else:
            answers[query] = {"status": "definite", "identities": [ranked[0][0]], "reason": None, "values": values}
    return answers


def outcome(record, answer, identities):
    ranking = record["forced_ranking"]
    if answer["status"] != "definite":
        return {"candidate_recall": None, "forced_top1": None, "mrr": None, "mrr_lower_bound": None, "mrr_upper_bound": None, "accepted_correct": None, "score_status": "unknown_gt_" + answer["status"]}
    matched = [item for item in ranking if identities[item["track_id"]]["status"] == "verified"]
    uncertain = [item for item in ranking if identities[item["track_id"]]["status"] != "verified"]
    candidate_positions = [index + 1 for index, item in enumerate(ranking) if identities[item["track_id"]]["status"] == "verified" and identities[item["track_id"]]["identity"] in answer["identities"]]
    candidate_recall = True if candidate_positions else (None if uncertain else False)
    top = ranking[0] if ranking else None
    top1 = False if top is None else (None if identities[top["track_id"]]["status"] != "verified" else identities[top["track_id"]]["identity"] in answer["identities"])
    mrr = 1.0 / candidate_positions[0] if candidate_positions and not uncertain else (None if uncertain else 0.0)
    mrr_lower = 1.0 / candidate_positions[0] if candidate_positions else 0.0
    uncertain_positions = [index + 1 for index, item in enumerate(ranking) if identities[item["track_id"]]["status"] != "verified"]
    uncertain_best = 1.0 / min(uncertain_positions) if uncertain_positions else 0.0
    mrr_upper = max(mrr_lower, uncertain_best)
    if not 0.0 <= mrr_lower <= mrr_upper <= 1.0:
        raise AssertionError("invalid MRR uncertainty interval")
    accepted = record["accepted_track_id"]
    accepted_correct = None if accepted is None or identities[accepted]["status"] != "verified" else identities[accepted]["identity"] in answer["identities"]
    return {"candidate_recall": candidate_recall, "forced_top1": top1, "mrr": mrr, "mrr_lower_bound": mrr_lower, "mrr_upper_bound": mrr_upper, "accepted_correct": accepted_correct, "score_status": "measured" if not uncertain else "partial_identity_uncertainty"}


def mean_known(rows, key):
    values = [row[key] for row in rows if row[key] is not None]
    return sum(values) / len(values) if values else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--private-dir", type=Path, required=True)
    parser.add_argument("--public-dir", type=Path, required=True)
    parser.add_argument("--public-input-dir", type=Path, required=True)
    parser.add_argument("--method", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise SystemExit("refusing to overwrite output directory")
    method, config = json.loads(args.method.read_text()), json.loads(args.config.read_text())
    rules = validate_rules(config["rules"])
    instances, gt_by_observation = load_ground_truth(args.private_dir, args.public_dir, args.public_input_dir, rules)
    assignments, assignment_diagnostics = constituent_assignments(method, gt_by_observation)
    identities = track_identities(method, assignments)
    calibration = json.loads((args.public_input_dir / "calibration_selfpose.json").read_text())["observations"]
    answers = ground_truth_answers(instances, heading(calibration["obs_001"]["self_pose"]["entity"]["orientation"]), rules)
    rows = []
    for record in method["records"]:
        answer = answers[record["query_id"]]
        row = {"variant": record["variant"], "query_id": record["query_id"], "gt_status": answer["status"], "gt_identities": answer["identities"], "gt_reason": answer.get("reason"), "accepted_track_id": record["accepted_track_id"], "public_abstain_reason": record["abstain_reason"], "public_relation_status": record["relation_status"], "public_relation_reason": record["relation_reason"]}
        row.update(outcome(record, answer, identities))
        rows.append(row)
    definite = [row for row in rows if row["gt_status"] == "definite"]
    accepted = [row for row in definite if row["accepted_track_id"] is not None]
    metrics = {"records": len(rows), "definite_gt_rows": len(definite), "candidate_recall": mean_known(definite, "candidate_recall"), "forced_top1": mean_known(definite, "forced_top1"), "mrr": mean_known(definite, "mrr"), "accepted_coverage": len(accepted) / len(definite) if definite else None, "wrong_accepted": sum(row["accepted_correct"] is False for row in accepted), "unknown_identity_score_rows": sum(row["score_status"] != "measured" for row in rows), "relation_unknown_rows": sum(row["public_relation_status"] == "unknown" for row in rows), "relation_reason_denominators": {reason: sum(row["public_relation_reason"] == reason for row in rows) for reason in sorted({row["public_relation_reason"] for row in rows if row["public_relation_reason"]})}}
    foreground_rows = [item["foreground"] for track in identities.values() for item in track["constituents"] if item.get("foreground")]
    metrics["known_score_counts"] = {key: sum(row[key] is not None for row in definite) for key in ("candidate_recall", "forced_top1", "mrr", "accepted_correct")}
    metrics["mrr_lower_bound"] = mean_known(definite, "mrr_lower_bound")
    metrics["mrr_upper_bound"] = mean_known(definite, "mrr_upper_bound")
    metrics["foreground_purity_mean"] = mean_known(foreground_rows, "purity")
    metrics["foreground_coverage_mean"] = mean_known(foreground_rows, "coverage")
    report = {"scope": "PRIVATE_EVALUATOR_ONLY_INDEPENDENT_OF_PUBLIC_FOREGROUND", "taxonomy_version": "a1-taxonomy-v2-explicit-assets", "taxonomy": TAXONOMY, "rules": {"constituent_assignment": "deterministic_maximum_weight_one_to_one_class_matched_iou_ge_0.5", "identity_rule": "verified only when every constituent matches one distinct identity; unmatched is uncertain and distinct identities are contamination", "relation": "private-mask observed-surface configured voxels; obs001 heading; bidirectional 10th-percentile gap; configured ambiguity and side margins widened by private voxel uncertainty"}, "input_hashes": {"method": sha256(args.method), "config": sha256(args.config), "private_instances": sha256(args.private_dir / "instances.json"), "public_calibration": sha256(args.public_input_dir / "calibration_selfpose.json")}, "ground_truth_instances": list(instances.values()), "ground_truth_answers": answers, "constituent_assignments": assignment_diagnostics, "track_identities": identities, "rows": rows, "metrics": metrics}
    args.output_dir.mkdir(parents=True)
    (args.output_dir / "evaluation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(metrics))


if __name__ == "__main__":
    main()
