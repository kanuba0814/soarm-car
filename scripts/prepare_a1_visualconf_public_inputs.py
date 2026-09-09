#!/usr/bin/env python3
"""Sanitize frozen detections and acquisition gates into the public A1 input boundary."""
import argparse
import json
import math
from pathlib import Path

from a1_visualconf.public import sha256


MOUNTING = {
    "base_to_camera_translation_m": [.180, 0.0, .2263],
    "optical_to_base_rows": [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]],
    "description": "declared base-to-camera mount; optical +Z forward, +X right, +Y down",
}


def tilt_degrees(quaternion):
    x, y, z, w = (float(quaternion[key]) for key in ("x", "y", "z", "w"))
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0:
        return float("inf")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--public-dir", type=Path, required=True)
    parser.add_argument("--frozen-detections", type=Path, required=True)
    parser.add_argument("--acquisition-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--tilt-limit-deg", type=float, default=5.0)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise SystemExit("refusing to overwrite output directory")
    manifest = json.loads((args.public_dir / "manifest.json").read_text())
    frozen = json.loads(args.frozen_detections.read_text())
    capture = json.loads((args.acquisition_dir / "capture.json").read_text())
    stream = json.loads((args.acquisition_dir / "raw_native_dynamic_pose_stream.json").read_text())["records"]
    capture_by_id = {item["id"]: item for item in capture["viewpoints"]}
    frozen_by_id = {Path(item["input"]["path"]).parent.name: item for item in frozen["frames"]}
    detections, gates, calibration = {}, {}, {}
    for observation in manifest["observations"]:
        observation_id = observation["observation_id"]
        capture_record = capture_by_id[observation_id]
        stamp = capture_record["rgb_stamp_ns"]
        samples = [item for item in stream if stamp - 250_000_000 <= item["source_stamp_ns"] <= stamp + 250_000_000]
        tilts = [tilt_degrees(item["entity"]["orientation"]) for item in samples]
        source_time_ok = observation["rgb_stamp_ns"] == capture_record["rgb_stamp_ns"] and observation["depth_stamp_ns"] == capture_record["depth_stamp_ns"]
        maximum_tilt = max(tilts, default=float("inf"))
        eligible = source_time_ok and bool(samples) and maximum_tilt <= args.tilt_limit_deg
        gates[observation_id] = {"opaque_observation_id": observation_id, "source_time_valid": source_time_ok, "window_sample_count": len(samples), "max_window_tilt_deg": maximum_tilt, "eligible": eligible, "reason": None if eligible else "invalid_physical_placement"}
        calibration[observation_id] = {"opaque_observation_id": observation_id, "self_pose": observation["self_pose"], "camera_mounting": MOUNTING}
        sanitized = []
        for index, box in enumerate(frozen_by_id.get(observation_id, {}).get("raw_boxes", [])):
            if box.get("label") in ("chair", "dining table", "couch") and float(box.get("confidence", 0)) >= .1:
                sanitized.append({"detection_index": index, "label": box["label"], "confidence": float(box["confidence"]), "box_xyxy": [float(value) for value in box["xyxy_pixels"]]})
        detections[observation_id] = sanitized
    args.output_dir.mkdir(parents=True)
    (args.output_dir / "detections.json").write_text(json.dumps({"scope": "PUBLIC_SANITIZED_DETECTIONS", "observations": detections}, indent=2) + "\n")
    (args.output_dir / "gates.json").write_text(json.dumps({"scope": "PUBLIC_VALIDATED_GATES", "observations": gates}, indent=2) + "\n")
    (args.output_dir / "calibration_selfpose.json").write_text(json.dumps({"scope": "PUBLIC_CALIBRATION_AND_SELFPOSE", "observations": calibration}, indent=2) + "\n")
    (args.output_dir / "input_hashes.json").write_text(json.dumps({"public_manifest_sha256": sha256(args.public_dir / "manifest.json"), "frozen_detections_sha256": sha256(args.frozen_detections), "acquisition_capture_sha256": sha256(args.acquisition_dir / "capture.json"), "acquisition_pose_stream_sha256": sha256(args.acquisition_dir / "raw_native_dynamic_pose_stream.json")}, indent=2) + "\n")
    print(json.dumps({"observations": len(gates), "eligible": sum(item["eligible"] for item in gates.values()), "sanitized_detections": sum(len(item) for item in detections.values())}))


if __name__ == "__main__":
    main()
