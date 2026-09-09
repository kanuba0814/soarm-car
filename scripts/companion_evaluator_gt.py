#!/usr/bin/env python3
"""Decode evaluator-only segmentation labels and export separated public/private survey packages."""
import argparse
import json
import shutil
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def decode_labels(frame):
    metadata = json.loads((frame / "labels_map.json").read_text())
    raw = (frame / "labels_map.raw").read_bytes()
    if metadata["encoding"] != "rgb8" or metadata["step"] != metadata["width"] * 3 or len(raw) != metadata["height"] * metadata["step"]:
        raise RuntimeError(f"unsupported labels_map encoding/layout: {metadata}")
    pixels = np.frombuffer(raw, np.uint8).reshape(metadata["height"], metadata["width"], 3)
    if not np.array_equal(pixels[:, :, 0], pixels[:, :, 1]) or not np.array_equal(pixels[:, :, 0], pixels[:, :, 2]):
        raise RuntimeError("ambiguous RGB label encoding: channels are not replicated instance IDs")
    return pixels[:, :, 0], metadata


def source_class(name):
    lower = name.lower()
    if "chair" in lower:
        return "chair"
    if "table" in lower or "bench" in lower:
        return "dining table"
    if "sofa" in lower:
        return "couch"
    return None


def bbox(mask):
    rows, columns = np.nonzero(mask)
    return [int(columns.min()), int(rows.min()), int(columns.max() + 1), int(rows.max() + 1)]


def iou(first, second):
    left, top = max(first[0], second[0]), max(first[1], second[1])
    right, bottom = min(first[2], second[2]), min(first[3], second[3])
    overlap = max(0, right - left) * max(0, bottom - top)
    union = (first[2] - first[0]) * (first[3] - first[1]) + (second[2] - second[0]) * (second[3] - second[1]) - overlap
    return overlap / union if union else 0.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--yolo-results", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise SystemExit(f"Refusing to overwrite output: {args.output_dir}")
    capture = json.loads((args.input_dir / "capture.json").read_text())
    labels = json.loads((args.input_dir / "private_instance_labels.json").read_text())["labels"]
    label_map = {item["label_id"]: item for item in labels}
    yolo = json.loads(args.yolo_results.read_text())
    yolo_by_observation = {Path(item["input"]["path"]).parent.name: item for item in yolo["frames"]}
    public_root, private_root = args.output_dir / "public", args.output_dir / "private"
    public_root.mkdir(parents=True); private_root.mkdir(); (private_root / "masks").mkdir()
    public_observations, private_observations, summary = [], [], []
    for record in capture["viewpoints"]:
        observation_id = record["id"]
        frame = args.input_dir / "frames" / observation_id
        values, metadata = decode_labels(frame)
        public_frame = public_root / observation_id; public_frame.mkdir()
        for filename in ("rgb.png", "depth_m_float32.npy", "depth_invalid_mask.npy", "camera_info.json"):
            shutil.copy2(frame / filename, public_frame / filename)
        public_observations.append({"observation_id": observation_id, "rgb_stamp_ns": record["rgb_stamp_ns"], "depth_stamp_ns": record["depth_stamp_ns"], "self_pose": record["native_pose_at_image"], "files": [filename for filename in ("rgb.png", "depth_m_float32.npy", "depth_invalid_mask.npy", "camera_info.json")]})
        visible = []
        overlay = Image.open(frame / "rgb.png").convert("RGB"); draw = ImageDraw.Draw(overlay)
        for label_id in np.unique(values):
            if label_id == 0:
                continue
            if int(label_id) not in label_map:
                raise RuntimeError(f"private label {int(label_id)} has no stable source mapping")
            mask = values == label_id
            pixels = int(mask.sum())
            if pixels < 100:
                continue
            item = label_map[int(label_id)]
            entry = {"label_id": int(label_id), "source_name": item["source_name"], "source_uri": item["source_uri"], "source_pose": item["source_pose"], "reporting_class": source_class(item["source_name"]), "visible_pixels": pixels, "bbox_xyxy": bbox(mask)}
            visible.append(entry)
            mask_dir = private_root / "masks" / observation_id
            mask_dir.mkdir(exist_ok=True)
            Image.fromarray((mask.astype(np.uint8) * 255), mode="L").save(mask_dir / f"{int(label_id):03d}.png")
            draw.rectangle(entry["bbox_xyxy"], outline=(255, 0, 255), width=2)
        overlay.save(private_root / f"{observation_id}_gt_overlay.png")
        private_observations.append({"observation_id": observation_id, "labels_metadata": metadata, "visible_instances": visible})
        predictions = yolo_by_observation.get(observation_id, {}).get("raw_boxes", [])
        eligible_gt = [item for item in visible if item["reporting_class"]]
        matched_gt, matches = set(), []
        for prediction in sorted(predictions, key=lambda item: item["confidence"], reverse=True):
            candidates = [(iou(prediction["xyxy_pixels"], target["bbox_xyxy"]), index) for index, target in enumerate(eligible_gt) if target["reporting_class"] == prediction["label"] and index not in matched_gt]
            if candidates:
                score, index = max(candidates)
                if score >= .5:
                    matched_gt.add(index); matches.append({"prediction_label": prediction["label"], "confidence": prediction["confidence"], "gt_label_id": eligible_gt[index]["label_id"], "iou": score})
        summary.append({"observation_id": observation_id, "visible_gt_reporting_instances_denominator": len(eligible_gt), "yolo_raw_boxes": len(predictions), "iou_threshold": .5, "class_matched_iou_matches": matches, "warning": "Detector-vs-GT visibility accounting only; not a relation, navigation, or accuracy score."})
    (public_root / "manifest.json").write_text(json.dumps({"scope": "PUBLIC_POLICY_INPUT_ONLY", "observations": public_observations, "forbidden": ["labels_map", "source instance names", "source poses", "private GT"]}, indent=2) + "\n")
    (private_root / "instances.json").write_text(json.dumps({"scope": "EVALUATOR_PRIVATE_ONLY", "observations": private_observations}, indent=2) + "\n")
    (private_root / "detector_gt_visibility.json").write_text(json.dumps({"scope": "EVALUATOR_PRIVATE_ONLY", "summary": summary}, indent=2) + "\n")
    print(json.dumps({"observations": len(public_observations), "visible_instances": sum(len(item["visible_instances"]) for item in private_observations), "public_has_labels": False}))


if __name__ == "__main__":
    main()
