#!/usr/bin/env python3
"""Run a boxes-only fixed-vocabulary YOLO-World smoke on saved companion RGB frames."""
import argparse
import hashlib
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

MODEL_URL = "https://github.com/ultralytics/assets/releases/download/v8.4.0/yolov8s-worldv2.pt"
MODEL_RELEASE_API = "https://api.github.com/repos/ultralytics/assets/releases/tags/v8.4.0"
MODEL_ASSET_ID = 340060641
MODEL_SIZE_BYTES = 25923032
MODEL_SHA256 = "9b2c17ab6124a913e9b3a5c170617920d91b0f01111a8479da69f00e2cf27792"
HOUSEHOLD_CLASSES = {
    "bed", "chair", "clock", "couch", "dining table", "laptop", "microwave",
    "oven", "potted plant", "refrigerator", "sink", "toaster", "tv", "vase",
}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--model",
        type=Path,
        default=Path("/home/gxxl/soarm-car/artifacts/perception-dev/models/yolov8s-worldv2.pt"),
    )
    parser.add_argument("--confidence", type=float, default=0.10)
    return parser.parse_args()


def main():
    args = parse_args()
    frames = sorted(args.input_dir.rglob("rgb.png"))
    if not frames:
        frames = sorted(args.input_dir.glob("*_rgb.png"))
    if len(frames) < 3:
        raise SystemExit("Expected at least three saved RGB PNGs")
    if args.output_dir.exists():
        raise SystemExit(f"Refusing to overwrite output directory: {args.output_dir}")
    if not args.model.is_file() or args.model.stat().st_size != MODEL_SIZE_BYTES:
        raise SystemExit(f"Unexpected model file: {args.model}")
    if sha256(args.model) != MODEL_SHA256:
        raise SystemExit(f"Model SHA-256 mismatch: {args.model}")

    import cv2
    import numpy
    import PIL
    import torch
    import ultralytics
    from ultralytics import YOLOWorld

    args.output_dir.mkdir(parents=True)
    model = YOLOWorld(str(args.model))
    device = 0 if torch.cuda.is_available() else "cpu"
    records = []
    for frame in frames:
        started = time.perf_counter()
        result = model.predict(str(frame), conf=args.confidence, device=device, verbose=False)[0]
        wall_ms = (time.perf_counter() - started) * 1000
        names = result.names
        boxes = []
        for box in result.boxes:
            class_id = int(box.cls.item())
            label = str(names[class_id])
            boxes.append({
                "class_id": class_id,
                "label": label,
                "confidence": float(box.conf.item()),
                "xyxy_pixels": [float(value) for value in box.xyxy[0].tolist()],
                "household_reporting_class": label in HOUSEHOLD_CLASSES,
            })
        overlay_path = args.output_dir / f"{frame.parent.name}_{frame.stem}_boxes.png"
        if not cv2.imwrite(str(overlay_path), result.plot()):
            raise RuntimeError(f"Could not save overlay: {overlay_path}")
        records.append({
            "input": {"path": str(frame), "sha256": sha256(frame)},
            "overlay": {"path": str(overlay_path), "sha256": sha256(overlay_path)},
            "wall_latency_ms": wall_ms,
            "ultralytics_speed_ms": {key: float(value) for key, value in result.speed.items()},
            "raw_boxes": boxes,
            "raw_box_count": len(boxes),
            "household_reporting_boxes": [box for box in boxes if box["household_reporting_class"]],
        })
    report = {
        "run_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "scope": "Boxes-only pretrained fixed-COCO-vocabulary smoke on saved RGB frames; not ground truth, accuracy, navigation, or task-success evidence.",
        "model": {
            "url": MODEL_URL,
            "release_api": MODEL_RELEASE_API,
            "release_asset_id": MODEL_ASSET_ID,
            "expected_bytes": MODEL_SIZE_BYTES,
            "sha256": MODEL_SHA256,
            "local_path": str(args.model),
        },
        "runtime": {
            "python": sys.version.split()[0],
            "torch": torch.__version__,
            "cuda_available": torch.cuda.is_available(),
            "cuda_device": torch.cuda.get_device_name(0) if torch.cuda.is_available() else None,
            "ultralytics": ultralytics.__version__,
            "opencv": cv2.__version__,
            "numpy": numpy.__version__,
            "pillow": PIL.__version__,
        },
        "inference": {"confidence_threshold": args.confidence, "device": device, "custom_vocabulary": False},
        "household_reporting_classes": sorted(HOUSEHOLD_CLASSES),
        "frames": records,
        "license_caveat": "Ultralytics is AGPL-3.0 unless separately commercially licensed; this local smoke makes no deployment claim.",
    }
    (args.output_dir / "results.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"frames": len(records), "raw_boxes": sum(record["raw_box_count"] for record in records), "output": str(args.output_dir)}, sort_keys=True))


if __name__ == "__main__":
    main()
