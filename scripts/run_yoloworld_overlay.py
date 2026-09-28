#!/usr/bin/env python3
import argparse
import contextlib
import json
import time
from pathlib import Path

import cv2
import numpy as np
import torch
from PIL import Image
from transformers import Sam2Model, Sam2Processor
from ultralytics import YOLOWorld


COLORS = [(255, 80, 80), (80, 220, 120), (80, 160, 255), (240, 190, 70)]


def overlay_mask(image, mask, color, alpha=0.38):
    output = image.copy()
    selected = mask.astype(bool)
    output[selected] = (
        output[selected].astype(np.float32) * (1 - alpha)
        + np.asarray(color, dtype=np.float32) * alpha
    ).astype(np.uint8)
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("frames", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--yolo-model", required=True)
    parser.add_argument("--sam-model", type=Path)
    parser.add_argument("--classes", nargs="+", default=None,
                        help="Custom open-vocabulary classes. Omit to use built-in COCO classes.")
    parser.add_argument("--builtin-class", default="bottle",
                        help="Keep only this built-in class when --classes is omitted.")
    parser.add_argument("--confidence", type=float, default=0.20)
    parser.add_argument("--iou", type=float, default=0.50)
    parser.add_argument("--fps", type=float, default=1.0)
    parser.add_argument("--device", default="0")
    args = parser.parse_args()

    model = YOLOWorld(args.yolo_model)
    if args.classes:
        model.set_classes(args.classes)
    device = torch.device("cuda:0" if args.device != "cpu" and torch.cuda.is_available() else "cpu")

    sam_processor = None
    sam_model = None
    if args.sam_model:
        sam_processor = Sam2Processor.from_pretrained(args.sam_model, local_files_only=True)
        sam_model = Sam2Model.from_pretrained(args.sam_model, local_files_only=True).to(device).eval()

    frame_paths = sorted(args.frames.glob("*.png"))
    if not frame_paths:
        raise RuntimeError(f"No PNG frames found in {args.frames}")
    first = cv2.imread(str(frame_paths[0]))
    height, width = first.shape[:2]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(args.output), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (width, height)
    )
    if not writer.isOpened():
        raise RuntimeError(f"Could not open {args.output}")

    frame_results = []
    if device.type == "cuda":
        torch.cuda.reset_peak_memory_stats()
    started = time.perf_counter()
    for frame_index, frame_path in enumerate(frame_paths, start=1):
        if device.type == "cuda":
            torch.cuda.synchronize()
        frame_started = time.perf_counter()
        bgr = cv2.imread(str(frame_path))
        prediction = model.predict(
            source=bgr,
            conf=args.confidence,
            iou=args.iou,
            imgsz=640,
            device=args.device,
            half=device.type == "cuda",
            verbose=False,
        )[0]
        boxes = prediction.boxes.xyxy.detach().cpu() if prediction.boxes is not None else torch.empty((0, 4))
        scores = prediction.boxes.conf.detach().cpu() if prediction.boxes is not None else torch.empty(0)
        class_ids = prediction.boxes.cls.detach().cpu().int().tolist() if prediction.boxes is not None else []
        labels = [prediction.names.get(index, str(index)) for index in class_ids]
        if not args.classes:
            keep = [index for index, label in enumerate(labels) if label == args.builtin_class]
            boxes = boxes[keep]
            scores = scores[keep]
            labels = [labels[index] for index in keep]

        image = Image.open(frame_path).convert("RGB")
        masks = []
        if len(boxes) and sam_model is not None and sam_processor is not None:
            sam_inputs = sam_processor(images=image, input_boxes=[boxes.tolist()], return_tensors="pt").to(device)
            autocast = (
                torch.autocast(device_type="cuda", dtype=torch.float16)
                if device.type == "cuda"
                else contextlib.nullcontext()
            )
            with torch.inference_mode(), autocast:
                sam_outputs = sam_model(**sam_inputs)
            processed_masks = sam_processor.post_process_masks(
                sam_outputs.pred_masks.cpu(), sam_inputs["original_sizes"]
            )[0]
            iou_scores = sam_outputs.iou_scores.cpu()[0]
            for detection_index in range(len(boxes)):
                candidate_index = int(torch.argmax(iou_scores[detection_index]).item())
                masks.append(processed_masks[detection_index, candidate_index].numpy() > 0)

        annotated = np.asarray(image).copy()
        detections = []
        for detection_index, (box, score, label) in enumerate(zip(boxes, scores, labels)):
            color = COLORS[detection_index % len(COLORS)]
            if detection_index < len(masks):
                annotated = overlay_mask(annotated, masks[detection_index], color)
            x1, y1, x2, y2 = [int(round(value)) for value in box.tolist()]
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            caption = f"{label} {float(score):.2f}"
            cv2.rectangle(annotated, (x1, max(0, y1 - 22)), (min(width, x1 + 9 * len(caption)), y1), color, -1)
            cv2.putText(annotated, caption, (x1 + 3, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 1, cv2.LINE_AA)
            detections.append({"label": label, "score": float(score), "box_xyxy": box.tolist()})

        cv2.putText(
            annotated,
            f"YOLO-World V2{' + SAM2' if sam_model is not None else ''} | DEV visual smoke | frame {frame_index}/{len(frame_paths)}",
            (10, height - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA,
        )
        writer.write(cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
        if device.type == "cuda":
            torch.cuda.synchronize()
        frame_results.append({"frame": frame_path.name, "seconds": time.perf_counter() - frame_started, "detections": detections})
        print(f"[{frame_index}/{len(frame_paths)}] {frame_path.name}: {len(detections)} detections", flush=True)

    writer.release()
    report = {
        "purpose": "DEV visual smoke test; not a scored benchmark",
        "device": str(device),
        "classes": args.classes or [args.builtin_class],
        "class_source": "custom_text" if args.classes else "weight_builtin",
        "confidence": args.confidence,
        "frames": len(frame_paths),
        "elapsed_seconds": time.perf_counter() - started,
        "peak_cuda_memory_mb": (
            torch.cuda.max_memory_allocated() / (1024 * 1024)
            if device.type == "cuda" else 0.0
        ),
        "results": frame_results,
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()
