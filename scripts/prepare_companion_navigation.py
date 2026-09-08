#!/usr/bin/env python3
"""Offline-only planner and documentation-completeness preflight for companion navigation research."""

import argparse
import json
import math
import re
import sys
from pathlib import Path

METHODS = ("baseline", "grounding_only", "viewpoint_only", "combined")
CASES = ("unique", "ambiguous", "occluded", "moved", "absent")
MAX_MATRIX_LIMIT = 10000
ID_PATTERN = re.compile(r"^[a-z0-9][a-z0-9_-]*$")


def reject_constant(value):
    raise ValueError("non-finite JSON value is not permitted: {}".format(value))


def reject_nonfinite_float(value):
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError("non-finite JSON value is not permitted: {}".format(value))
    return parsed


def reject_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key: {}".format(key))
        result[key] = value
    return result


def load_json(path):
    try:
        with Path(path).open("r", encoding="utf-8") as handle:
            return json.load(handle, object_pairs_hook=reject_duplicate_keys,
                             parse_constant=reject_constant,
                             parse_float=reject_nonfinite_float)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        raise ValueError("cannot read strict JSON {}: {}".format(path, error)) from error


def require_new_path(path):
    output = Path(path)
    if output.exists():
        raise ValueError("refusing to overwrite existing path: {}".format(output))
    if not output.parent.exists():
        raise ValueError("output parent does not exist: {}".format(output.parent))
    return output


def write_new_json(path, document):
    output = require_new_path(path)
    with output.open("x", encoding="utf-8") as handle:
        json.dump(document, handle, indent=2, sort_keys=True)
        handle.write("\n")


def is_plain_string(value):
    return isinstance(value, str) and bool(value.strip())


def require_identifier(value, label):
    if not is_plain_string(value) or not ID_PATTERN.fullmatch(value):
        raise ValueError("{} must be a nonempty lowercase identifier".format(label))


def require_unique_identifiers(values, label):
    if not isinstance(values, list) or not values:
        raise ValueError("{} must be a nonempty array".format(label))
    for value in values:
        require_identifier(value, label + " entry")
    if len(values) != len(set(values)):
        raise ValueError("{} entries must be unique".format(label))


def validate_config(config):
    if not isinstance(config, dict):
        raise ValueError("config must be an object")
    require_identifier(config.get("study_id"), "study_id")
    if config.get("recommended_focus") != "grounding+viewpoint":
        raise ValueError("recommended_focus must be grounding+viewpoint")
    if not is_plain_string(config.get("planned_only_note")):
        raise ValueError("planned_only_note must describe hypothetical pilot slots")
    directions = config.get("proposed_directions")
    if not isinstance(directions, list) or len(directions) != 3:
        raise ValueError("proposed_directions must contain exactly three proposed directions")
    direction_ids = []
    for direction in directions:
        if not isinstance(direction, dict):
            raise ValueError("each proposed direction must be an object")
        require_identifier(direction.get("id"), "proposed direction id")
        if not is_plain_string(direction.get("description")):
            raise ValueError("each proposed direction needs a nonempty description")
        direction_ids.append(direction["id"])
    if len(direction_ids) != len(set(direction_ids)):
        raise ValueError("proposed direction ids must be unique")
    splits = config.get("scene_splits")
    if not isinstance(splits, dict) or set(splits) != {"train", "dev", "test"}:
        raise ValueError("scene_splits must contain exactly train, dev, and test")
    scenes = []
    for split in ("train", "dev", "test"):
        split_scenes = splits[split]
        require_unique_identifiers(split_scenes, "scene_splits.{}".format(split))
        scenes.extend(split_scenes)
    if len(scenes) != len(set(scenes)):
        raise ValueError("scene IDs must not appear in more than one split")
    seeds = config.get("seeds")
    if (not isinstance(seeds, list) or not seeds or
            any(isinstance(seed, bool) or not isinstance(seed, int) for seed in seeds) or
            len(seeds) != len(set(seeds))):
        raise ValueError("seeds must be a nonempty array of unique integers")
    limit = config.get("matrix_limit")
    if isinstance(limit, bool) or not isinstance(limit, int) or not 1 <= limit <= MAX_MATRIX_LIMIT:
        raise ValueError("matrix_limit must be an integer from 1 to {}".format(MAX_MATRIX_LIMIT))
    prerequisites = config.get("geometry_prerequisites")
    expected = {
        "rgb_depth_registration": "unknown",
        "rgb_depth_synchronization": "unknown",
        "pose_to_map_transform": "unknown",
        "map_bundle_identity": "unknown",
        "ground_truth": "draft_not_frozen",
    }
    if prerequisites != expected:
        raise ValueError("geometry_prerequisites must retain the required initial unknown/draft states")
    count = len(scenes) * len(seeds) * len(METHODS) * len(CASES)
    if count > limit:
        raise ValueError("episode matrix size {} exceeds matrix_limit {}".format(count, limit))
    return count


def build_plan(config):
    count = validate_config(config)
    episodes = []
    for split in ("train", "dev", "test"):
        for scene_id in config["scene_splits"][split]:
            for seed in config["seeds"]:
                for method in METHODS:
                    for case in CASES:
                        episode_id = "{}--{}--s{}--{}--{}".format(
                            split, scene_id, seed, method, case)
                        episodes.append({
                            "case": case,
                            "episode_id": episode_id,
                            "method": method,
                            "scene_id": scene_id,
                            "seed": seed,
                            "split": split,
                            "status": "planned",
                        })
    return {
        "episode_count": count,
        "episodes": episodes,
        "kind": "companion_navigation_draft_episode_matrix",
        "planned_only_note": config["planned_only_note"],
        "recommended_focus": config["recommended_focus"],
        "study_id": config["study_id"],
    }


TEMPLATE = {
    "kind": "companion_navigation_geometry_evidence_manifest",
    "rgb_depth": {
        "registration_verified": False,
        "registration_evidence_files": [],
        "synchronization_verified": False,
        "synchronization_evidence_files": [],
    },
    "pose_map_transform": {
        "verified": False,
        "frame": "",
        "units": "",
        "evidence_files": [],
    },
    "map_bundle": {"id": "", "evidence_files": []},
    "ground_truth": {"frozen": False, "map_bundle_id": "", "evidence_files": []},
}


def evidence_files(value, label, manifest_dir, failures):
    if not isinstance(value, list) or not value:
        failures.append(label + " must name at least one evidence file")
        return
    if any(not is_plain_string(item) for item in value):
        failures.append(label + " must be unique nonempty strings")
        return
    if len(value) != len(set(value)):
        failures.append(label + " must be unique nonempty strings")
        return
    for item in value:
        relative_path = Path(item)
        evidence_path = manifest_dir / relative_path
        if (relative_path.is_absolute() or not evidence_path.is_file() or
                evidence_path.stat().st_size == 0):
            failures.append(label + " missing or empty: " + item)


def preflight(manifest_path):
    manifest = load_json(manifest_path)
    failures = []
    if not isinstance(manifest, dict) or manifest.get("kind") != TEMPLATE["kind"]:
        failures.append("kind must be " + TEMPLATE["kind"])
        manifest = manifest if isinstance(manifest, dict) else {}
    rgb_depth = manifest.get("rgb_depth")
    transform = manifest.get("pose_map_transform")
    map_bundle = manifest.get("map_bundle")
    ground_truth = manifest.get("ground_truth")
    if not isinstance(rgb_depth, dict):
        failures.append("rgb_depth must be an object")
        rgb_depth = {}
    for field, evidence_field in (("registration_verified", "registration_evidence_files"),
                                  ("synchronization_verified", "synchronization_evidence_files")):
        if rgb_depth.get(field) is not True:
            failures.append("rgb_depth.{} must be true".format(field))
        evidence_files(rgb_depth.get(evidence_field), "rgb_depth." + evidence_field,
                       Path(manifest_path).parent, failures)
    if not isinstance(transform, dict):
        failures.append("pose_map_transform must be an object")
        transform = {}
    if transform.get("verified") is not True:
        failures.append("pose_map_transform.verified must be true")
    if transform.get("frame") != "map":
        failures.append("pose_map_transform.frame must be map")
    if transform.get("units") != "meters":
        failures.append("pose_map_transform.units must be meters")
    evidence_files(transform.get("evidence_files"), "pose_map_transform.evidence_files",
                   Path(manifest_path).parent, failures)
    if not isinstance(map_bundle, dict):
        failures.append("map_bundle must be an object")
        map_bundle = {}
    if not is_plain_string(map_bundle.get("id")):
        failures.append("map_bundle.id must be a nonempty identity")
    evidence_files(map_bundle.get("evidence_files"), "map_bundle.evidence_files",
                   Path(manifest_path).parent, failures)
    if not isinstance(ground_truth, dict):
        failures.append("ground_truth must be an object")
        ground_truth = {}
    if ground_truth.get("frozen") is not True:
        failures.append("ground_truth.frozen must be true")
    if not is_plain_string(ground_truth.get("map_bundle_id")):
        failures.append("ground_truth.map_bundle_id must be a nonempty identity")
    elif ground_truth.get("map_bundle_id") != map_bundle.get("id"):
        failures.append("ground_truth.map_bundle_id must match map_bundle.id")
    evidence_files(ground_truth.get("evidence_files"), "ground_truth.evidence_files",
                   Path(manifest_path).parent, failures)
    return failures


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    plan_parser = commands.add_parser("plan", help="generate a draft episode matrix")
    plan_parser.add_argument("--config", required=True)
    plan_parser.add_argument("--output", required=True)
    template_parser = commands.add_parser("template", help="write a blocked-by-default evidence manifest")
    template_parser.add_argument("--output", required=True)
    preflight_parser = commands.add_parser("preflight", help="read-only documentation completeness check")
    preflight_parser.add_argument("--manifest", required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "plan":
            write_new_json(args.output, build_plan(load_json(args.config)))
            print("WROTE draft episode matrix: {}".format(args.output))
            return 0
        if args.command == "template":
            write_new_json(args.output, TEMPLATE)
            print("WROTE blocked-by-default evidence manifest template: {}".format(args.output))
            return 0
        failures = preflight(args.manifest)
    except (OSError, ValueError) as error:
        print("ERROR: " + str(error), file=sys.stderr)
        return 2
    if failures:
        print("BLOCKED: documentation completeness requirements are not met")
        for failure in failures:
            print("- " + failure)
        return 1
    print("PASS: documentation completeness only; this does not verify evidence correctness or permit execution.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
