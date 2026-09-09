#!/usr/bin/env python3
"""Run the DEV-only local VLM semantic selector through a conservative RGB-D grid gate."""
import argparse
import hashlib
import json
import re
import time
from pathlib import Path

import cv2
import numpy as np
import torch
from PIL import Image
from transformers import AutoModelForImageTextToText, AutoProcessor

from a1_visualconf.public import camera_transform
from vlm_grid.geometry import FREE, OCCUPIED, UNKNOWN, build_grid, inflate_occupied, reachable_approach


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parse_selection(raw, candidate_ids):
    try:
        parsed = json.loads(raw[raw.find("{"):raw.rfind("}") + 1])
    except (json.JSONDecodeError, ValueError):
        return None, "invalid_json"
    candidate_id = parsed.get("candidate_id")
    if candidate_id == "NO_MATCH":
        return parsed, None
    if candidate_id not in candidate_ids:
        return None, "unknown_candidate_id"
    if not isinstance(parsed.get("scene_description"), str):
        return None, "missing_scene_description"
    return parsed, None


def draw_candidates(image, candidates):
    canvas = np.asarray(image.convert("RGB"))[:, :, ::-1].copy()
    for candidate in candidates:
        left, top, right, bottom = (round(value) for value in candidate["box_xyxy"])
        color = (0, 220, 0) if candidate["target_status"] == "ok" else (0, 0, 255)
        cv2.rectangle(canvas, (left, top), (right, bottom), color, 2)
        cv2.putText(canvas, candidate["candidate_id"], (left, max(18, top - 5)), cv2.FONT_HERSHEY_SIMPLEX, .48, color, 1, cv2.LINE_AA)
    return Image.fromarray(canvas[:, :, ::-1])


def world_points(depth, info, transform, stride, max_depth):
    rows, columns = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    rows, columns = rows.ravel(), columns.ravel()
    z = depth[rows, columns]
    valid = np.isfinite(z) & (z > 0) & (z <= max_depth)
    rows, columns, z = rows[valid], columns[valid], z[valid]
    fx, fy, cx, cy = info["k"][0], info["k"][4], info["k"][2], info["k"][5]
    camera = np.column_stack(((columns - cx) * z / fx, (rows - cy) * z / fy, z))
    origin, rotation = transform
    return camera @ rotation.T + origin


def candidate_targets(depth, info, transform, detections):
    candidates = []
    fx, fy, cx, cy = info["k"][0], info["k"][4], info["k"][2], info["k"][5]
    origin, rotation = transform
    for detection in detections:
        left, top, right, bottom = [int(round(value)) for value in detection["box_xyxy"]]
        left, top = max(0, left), max(0, top)
        right, bottom = min(depth.shape[1], right), min(depth.shape[0], bottom)
        sample = depth[top:bottom, left:right]
        valid = np.isfinite(sample) & (sample > 0) & (sample <= 5.0)
        candidate = {"candidate_id": f"{detection['observation_id']}:{detection['detection_index']}", "label": detection["label"], "confidence": detection["confidence"], "box_xyxy": detection["box_xyxy"]}
        if valid.sum() < 20:
            candidate.update({"target_status": "no_valid_depth", "target_xy_m": None})
        else:
            pixels = np.argwhere(valid) + np.array([top, left])
            z = depth[pixels[:, 0], pixels[:, 1]]
            camera = np.column_stack(((pixels[:, 1] - cx) * z / fx, (pixels[:, 0] - cy) * z / fy, z))
            points = camera @ rotation.T + origin
            candidate.update({"target_status": "ok", "target_xy_m": np.median(points[:, :2], axis=0).tolist(), "depth_median_m": float(np.median(z))})
        candidates.append(candidate)
    return candidates


def render_grid(path, grid, origin, resolution, robot_xy, candidates, decisions):
    colors = np.zeros((*grid.shape, 3), dtype=np.uint8)
    colors[grid == UNKNOWN] = (80, 80, 80)
    colors[grid == FREE] = (240, 240, 240)
    colors[grid == OCCUPIED] = (30, 30, 30)
    scale = 3
    canvas = cv2.resize(colors, (grid.shape[1] * scale, grid.shape[0] * scale), interpolation=cv2.INTER_NEAREST)
    def point(xy, color, radius=3):
        cell = np.floor((np.asarray(xy) - origin) / resolution).astype(int)
        cv2.circle(canvas, (int((cell[0] + .5) * scale), int((cell[1] + .5) * scale)), radius, color, -1)
    point(robot_xy, (255, 0, 0), 4)
    for candidate in candidates:
        if candidate["target_xy_m"] is not None:
            point(candidate["target_xy_m"], (0, 165, 255), 3)
    for decision in decisions.values():
        route = decision.get("grid", {}).get("route_cells", [])
        for cell in route:
            cv2.circle(canvas, (int((cell[0] + .5) * scale), int((cell[1] + .5) * scale)), 1, (0, 200, 0), -1)
    cv2.imwrite(str(path), canvas)


def generate(model, processor, image, prompt, device, max_new_tokens):
    messages = [{"role": "user", "content": [{"type": "image"}, {"type": "text", "text": prompt}]}]
    text = processor.apply_chat_template(messages, add_generation_prompt=True)
    inputs = processor(text=text, images=[image], return_tensors="pt").to(device)
    start = time.perf_counter()
    with torch.inference_mode():
        generated = model.generate(**inputs, max_new_tokens=max_new_tokens, do_sample=False)
    latency = time.perf_counter() - start
    answer = processor.batch_decode(generated[:, inputs["input_ids"].shape[1]:], skip_special_tokens=True)[0]
    return answer, latency


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--public-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise SystemExit("refusing to overwrite output directory")
    config = json.loads(args.config.read_text())
    public = args.public_dir
    calibration = json.loads((public / "calibration_selfpose.json").read_text())["observations"]
    detections = json.loads((public / "detections.json").read_text())["observations"]
    device = "cuda" if torch.cuda.is_available() else "cpu"
    torch.cuda.reset_peak_memory_stats() if device == "cuda" else None
    load_start = time.perf_counter()
    processor = AutoProcessor.from_pretrained(args.model_dir, local_files_only=True)
    model = AutoModelForImageTextToText.from_pretrained(args.model_dir, dtype=torch.float16 if device == "cuda" else torch.float32, local_files_only=True).to(device).eval()
    load_seconds = time.perf_counter() - load_start
    args.output_dir.mkdir(parents=True)
    overlays = args.output_dir / "overlays"
    overlays.mkdir()
    records, latencies = [], []
    for observation_id in config["observations"]:
        root = public / observation_id
        image = Image.open(root / "rgb.png").convert("RGB")
        depth = np.load(root / "depth_m_float32.npy")
        info = json.loads((root / "camera_info.json").read_text())
        transform = camera_transform(calibration[observation_id]["self_pose"], calibration[observation_id]["camera_mounting"])
        source_xy = transform[0][:2]
        frame_detections = [dict(item, observation_id=observation_id) for item in detections[observation_id]]
        candidates = candidate_targets(depth, info, transform, frame_detections)
        boxed = draw_candidates(image, candidates)
        boxed.save(overlays / f"{observation_id}_candidates.png")
        raw_selection, select_latency = generate(model, processor, boxed, config["candidate_prompt"], device, config["model"]["max_new_tokens"])
        raw_description, description_latency = generate(model, processor, image, config["description_prompt"], device, config["model"]["max_new_tokens"])
        latencies.extend([select_latency, description_latency])
        parsed, parse_error = parse_selection(raw_selection, {item["candidate_id"] for item in candidates})
        points = world_points(depth, info, transform, config["grid"]["ray_stride_px"], config["grid"]["max_depth_m"])
        obstacles = np.abs(points[:, 2]) > config["grid"]["floor_clearance_m"]
        raw_grid, origin = build_grid(source_xy, points, obstacles, config["grid"]["resolution_m"], config["grid"]["robot_radius_m"])
        grid = inflate_occupied(raw_grid, round(config["grid"]["robot_radius_m"] / config["grid"]["resolution_m"]))
        grid_results = {}
        for candidate in candidates:
            if candidate["target_xy_m"] is None:
                grid_results[candidate["candidate_id"]] = {"status": "rejected", "reason": candidate["target_status"]}
            else:
                route, reason = reachable_approach(grid, origin, config["grid"]["resolution_m"], source_xy, candidate["target_xy_m"], config["grid"]["approach_radius_m"])
                grid_results[candidate["candidate_id"]] = {"status": "feasible", "grid": route} if route else {"status": "rejected", "reason": reason}
        feasible = [candidate for candidate in candidates if grid_results[candidate["candidate_id"]]["status"] == "feasible"]
        geometry_choice = min(feasible, key=lambda item: grid_results[item["candidate_id"]]["grid"]["route_length_m"])["candidate_id"] if feasible else "NO_FEASIBLE_CANDIDATE"
        vlm_choice = parsed["candidate_id"] if parsed else "INVALID_RESPONSE"
        hybrid = grid_results.get(vlm_choice, {"status": "rejected", "reason": parse_error or "no_match"})
        hybrid_choice = vlm_choice if hybrid["status"] == "feasible" else geometry_choice
        decisions = {"geometry_only": {"choice": geometry_choice}, "vlm_only": {"choice": vlm_choice, "parse_error": parse_error}, "hybrid_gated": {"choice": hybrid_choice, "vlm_gate": hybrid}}
        render_grid(overlays / f"{observation_id}_grid.png", grid, origin, config["grid"]["resolution_m"], source_xy, candidates, grid_results)
        records.append({"observation_id": observation_id, "raw_vlm_selection": raw_selection, "raw_vlm_description": raw_description, "selection_parse_error": parse_error, "parsed_selection": parsed, "candidates": candidates, "grid": {"frame": "per-observation RGB-D raycast in calibration self-pose world frame", "saved_sensor_map_used": False, "saved_sensor_map_reason": "no same-run camera/world transform observed", "shape_cells": list(grid.shape), "coverage": {"free_cells": int((grid == FREE).sum()), "occupied_or_padded_cells": int((grid == OCCUPIED).sum()), "unknown_cells": int((grid == UNKNOWN).sum())}, "candidate_results": grid_results}, "decisions": decisions, "latency_seconds": {"selection": select_latency, "description": description_latency}})
    result = {"scope": "DEV_ONLY_NO_ACTION_EXECUTION", "inputs": {"public_manifest_sha256": digest(public / "manifest.json"), "config_sha256": digest(args.config)}, "model": {"id": config["model"]["id"], "revision": config["model"]["revision"], "local_files": {path.name: digest(path) for path in sorted(args.model_dir.iterdir()) if path.is_file()}}, "runtime": {"device": device, "load_seconds": load_seconds, "peak_cuda_allocated_mib": torch.cuda.max_memory_allocated() / 2**20 if device == "cuda" else None, "per_query_latency_seconds": latencies}, "negative_checks": {"invalid_json": parse_selection("not json", {"obs_001:0"})[1], "unknown_candidate": parse_selection('{\"candidate_id\":\"invented\",\"scene_description\":\"x\"}', {"obs_001:0"})[1]}, "records": records, "replay": "python scripts/run_vlm_grid_demo.py --public-dir artifacts/companion-home-validation/evidence/vlm-grid-dev-v1/public-input --config config/vlm_grid_dev_v1.json --model-dir /home/gxxl/.cache/soarm-vlm-grid/models/smolvlm-500m --output-dir <new-output-dir>"}
    (args.output_dir / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"observations": len(records), "valid_json": sum(record["selection_parse_error"] is None for record in records), "hybrid_feasible": sum(record["decisions"]["hybrid_gated"]["vlm_gate"]["status"] == "feasible" for record in records), "load_seconds": load_seconds}))


if __name__ == "__main__":
    main()
