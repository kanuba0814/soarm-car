#!/usr/bin/env python3
"""Export FARM scene_state.pt into the benchmark's canonical inventory schema."""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

import torch


def load_official_consumer(repo: Path):
    viewer = repo / "scripts" / "view_scene_state.py"
    spec = importlib.util.spec_from_file_location("farm_view_scene_state", viewer)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load official FARM consumer: {viewer}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--state", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sequence-id", default="B06_20260901-181608")
    parser.add_argument("--source-stride", type=int, default=1)
    args = parser.parse_args()

    consumer = load_official_consumer(args.repo.resolve())
    state = consumer.load_state(args.state.resolve())
    means = state.get("means")
    if not isinstance(means, torch.Tensor) or means.ndim != 2 or means.shape[1] != 3:
        raise ValueError("FARM scene state has no Nx3 means tensor")

    object_ids = state.get("object_id")
    active = state.get("active")
    counts = state.get("count")
    image_ids = state.get("object_image_ids") or []
    confidence_maps = state.get("object_detection_category_conf") or []
    instances = []
    for index in range(int(means.shape[0])):
        if isinstance(active, torch.Tensor) and not bool(active[index].item()):
            continue
        confidence_map = confidence_maps[index] if index < len(confidence_maps) else {}
        ranked = sorted(
            ((str(name), float(score)) for name, score in (confidence_map or {}).items()),
            key=lambda item: (-item[1], item[0]),
        )
        canonical_name = ranked[0][0] if ranked else ""
        farm_id = int(object_ids[index].item()) if isinstance(object_ids, torch.Tensor) else index
        support = image_ids[index] if index < len(image_ids) else []
        instances.append(
            {
                "instance_id": f"farm-{farm_id:03d}",
                "canonical_name": canonical_name,
                "name_candidates": [
                    {"name": name, "confidence": score} for name, score in ranked[:5]
                ],
                "supporting_frames": [int(frame) * args.source_stride for frame in support],
                "supporting_detections": int(counts[index].item()) if isinstance(counts, torch.Tensor) else len(support),
                "centroid": [float(value) for value in means[index].tolist()],
            }
        )

    payload = {
        "schema": "soarm.inventory_prediction.v1",
        "sequence_id": args.sequence_id,
        "system": "FARM",
        "run": {
            "scene_state": str(args.state.resolve()),
            "source_stride": args.source_stride,
            "captioning_enabled": False,
            "dino_merge_enabled": False,
            "warning": "CPU compatibility run; RGB-depth registration remains unverified.",
        },
        "instances": instances,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"exported {len(instances)} active FARM instances to {args.output}")


if __name__ == "__main__":
    main()
