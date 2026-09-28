#!/usr/bin/env python3
"""Run ConceptGraphs' streamlined YOLO-World/MobileSAM/OpenCLIP front end on one image."""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import open_clip
import torch
from PIL import Image
from ultralytics import SAM, YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path)
    parser.add_argument("classes", type=Path)
    parser.add_argument("yolo_weights", type=Path)
    parser.add_argument("sam_weights", type=Path)
    parser.add_argument("clip_weights", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--confidence", type=float, default=0.1)
    args = parser.parse_args()

    started = time.perf_counter()
    classes = [
        line.strip()
        for line in args.classes.read_text(encoding="utf-8").splitlines()
        if line.strip() not in {"wall", "floor", "ceiling"}
    ]
    detector = YOLO(str(args.yolo_weights))
    detector.set_classes(classes)
    segmenter = SAM(str(args.sam_weights))
    clip_model, _, clip_preprocess = open_clip.create_model_and_transforms(
        "ViT-H-14", pretrained=str(args.clip_weights), device="cpu"
    )

    detection = detector.predict(str(args.image), conf=args.confidence, device="cpu", verbose=False)[0]
    boxes = detection.boxes.xyxy
    segmentation = segmenter.predict(
        str(args.image), bboxes=boxes, device="cpu", verbose=False
    )[0]

    image_bgr = cv2.imread(str(args.image))
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    masks = (
        segmentation.masks.data.cpu().numpy()
        if segmentation.masks is not None
        else np.empty((0, image_bgr.shape[0], image_bgr.shape[1]))
    )
    class_ids = detection.boxes.cls.cpu().numpy().astype(int)
    confidences = detection.boxes.conf.cpu().numpy()
    box_values = boxes.cpu().numpy()

    crop_features = []
    image_height, image_width = image_rgb.shape[:2]
    for box in box_values:
        x1, y1, x2, y2 = np.rint(box).astype(int)
        x1 = int(np.clip(x1, 0, image_width))
        x2 = int(np.clip(x2, 0, image_width))
        y1 = int(np.clip(y1, 0, image_height))
        y2 = int(np.clip(y2, 0, image_height))
        if x2 <= x1 or y2 <= y1:
            crop_features.append(None)
            continue
        crop = Image.fromarray(image_rgb[y1:y2, x1:x2])
        with torch.inference_mode():
            feature = clip_model.encode_image(clip_preprocess(crop).unsqueeze(0))
            feature /= feature.norm(dim=-1, keepdim=True)
        crop_features.append(feature.shape[-1])

    annotated = image_bgr.copy()
    for index, (box, class_id, confidence) in enumerate(zip(box_values, class_ids, confidences)):
        color = ((37 * index) % 255, (97 * index + 80) % 255, (173 * index + 40) % 255)
        if index < len(masks):
            selected = masks[index] > 0.5
            annotated[selected] = (
                annotated[selected].astype(np.float32) * 0.55
                + np.asarray(color, dtype=np.float32) * 0.45
            ).astype(np.uint8)
        x1, y1, x2, y2 = np.rint(box).astype(int)
        label = f"{classes[class_id]} {confidence:.2f}"
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
        cv2.putText(annotated, label, (x1, max(15, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), annotated)
    result = {
        "schema": "soarm.conceptgraphs_frontend_smoke.v1",
        "image": str(args.image),
        "detections": len(box_values),
        "clip_feature_dimensions": crop_features,
        "elapsed_seconds": time.perf_counter() - started,
        "note": "2D front-end smoke only; not a de-duplicated inventory or 3D ConceptGraphs result.",
    }
    args.output.with_suffix(".json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
