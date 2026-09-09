#!/usr/bin/env python3
"""Run the A1 public-only RGB-D to relation-record consumer path."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from a1_visualconf.public import add_sensitivity, associate, camera_transform, foreground, make_records, quaternion_matrix, score_tracks, sha256, validate_rules


def heading(quaternion):
    rotation = quaternion_matrix(quaternion).T
    forward = rotation @ np.array([1.0, 0.0, 0.0])
    return math.atan2(forward[1], forward[0])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--public-dir", type=Path, required=True)
    parser.add_argument("--public-input-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise SystemExit("refusing to overwrite output directory")
    manifest = json.loads((args.public_dir / "manifest.json").read_text())
    config = json.loads(args.config.read_text())
    rules = validate_rules(config["rules"])
    public_input = args.public_input_dir
    detections_by_id = json.loads((public_input / "detections.json").read_text())["observations"]
    gates = json.loads((public_input / "gates.json").read_text())["observations"]
    calibration = json.loads((public_input / "calibration_selfpose.json").read_text())["observations"]
    frames, detections, excluded = {}, [], []
    for observation in manifest["observations"]:
        observation_id = observation["observation_id"]
        gate = gates[observation_id]
        if not gate["eligible"]:
            excluded.append({"observation_id": observation_id, "reason": gate["reason"]})
            continue
        root = args.public_dir / observation_id
        depth = np.load(root / "depth_m_float32.npy")
        invalid = np.load(root / "depth_invalid_mask.npy")
        info = json.loads((root / "camera_info.json").read_text())
        if info.get("depth_unit") != "m" or depth.dtype != np.float32 or depth.shape != invalid.shape:
            raise RuntimeError(f"public RGB-D invariant failed for {observation_id}")
        transform = camera_transform(calibration[observation_id]["self_pose"], calibration[observation_id]["camera_mounting"])
        frames[observation_id] = {"depth": depth, "invalid": invalid, "info": info, "transform": transform}
        for item in detections_by_id.get(observation_id, []):
            proposed = foreground(depth, invalid, info, transform, item["box_xyxy"], rules)
            if not proposed["failure"]:
                add_sensitivity(depth, invalid, info, transform, item["box_xyxy"], proposed, rules)
            center = transform[0][:2]
            target = np.asarray(proposed.get("xy_median_m", center), dtype=float)
            detections.append({"observation_id": observation_id, "detection_index": item["detection_index"], "label": item["label"], "confidence": item["confidence"], "box_xyxy": item["box_xyxy"], "camera_center_xy_m": center.tolist(), "camera_bearing_rad": float(math.atan2(target[1] - center[1], target[0] - center[0])), "foreground": proposed})
    tracks, association_audit = associate(detections, frames, rules)
    score_tracks(tracks, rules)
    reference = calibration.get("obs_001")
    if reference is None:
        reference_heading = 0.0
        heading_status = "missing_reference_obs001"
    else:
        reference_heading = heading(reference["self_pose"]["entity"]["orientation"])
        heading_status = "measured_obs001_heading"
    records = make_records(config, tracks, reference_heading)
    args.output_dir.mkdir(parents=True)
    output = {"scope": "PUBLIC_A1_GEOMETRY_ASSOCIATION_RELATION_ONLY", "input_boundary": {"forbidden": ["private masks", "source metadata", "survey paths", "private evaluator"], "public_input_hashes": {"manifest": sha256(args.public_dir / "manifest.json"), "detections": sha256(public_input / "detections.json"), "gates": sha256(public_input / "gates.json"), "calibration_selfpose": sha256(public_input / "calibration_selfpose.json"), "config": sha256(args.config)}}, "gates": gates, "excluded_observations": excluded, "detections": detections, "tracks": tracks, "association_audit": association_audit, "reference_heading": {"status": heading_status, "radians": reference_heading}, "records": records, "replay": f"python3 scripts/run_a1_visualconf_public.py --public-dir {args.public_dir} --public-input-dir {public_input} --config {args.config} --output-dir <new-output-dir>"}
    (args.output_dir / "method.json").write_text(json.dumps(output, indent=2) + "\n")
    print(json.dumps({"components": sum(not item["foreground"]["failure"] for item in detections), "detections": len(detections), "tracks": len(tracks), "records": len(records), "accepted": sum(item["accepted_track_id"] is not None for item in records)}))


if __name__ == "__main__":
    main()
