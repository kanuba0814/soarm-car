#!/usr/bin/env python3
"""Bounded v2 format-compliance replay on the four public DEV RGB images."""
import argparse
import hashlib
import json
import time
from pathlib import Path

import torch
from PIL import Image
from transformers import AutoModelForImageTextToText, AutoProcessor

from run_vlm_grid_demo import candidate_targets, draw_candidates, parse_selection
from a1_visualconf.public import camera_transform


def sha256_text(value):
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def inspect_input(processor, image, prompt):
    messages = [{"role": "user", "content": [{"type": "image"}, {"type": "text", "text": prompt}]}]
    template = processor.apply_chat_template(messages, add_generation_prompt=True)
    inputs = processor(text=template, images=[image], return_tensors="pt")
    return {
        "source_mode": image.mode,
        "source_size_px": list(image.size),
        "chat_template_sha256": sha256_text(template),
        "tensor_shapes": {key: list(value.shape) for key, value in inputs.items() if hasattr(value, "shape")}
    }


def generate(model, processor, image, prompt, device, max_tokens):
    messages = [{"role": "user", "content": [{"type": "image"}, {"type": "text", "text": prompt}]}]
    template = processor.apply_chat_template(messages, add_generation_prompt=True)
    inputs = processor(text=template, images=[image], return_tensors="pt").to(device)
    start = time.perf_counter()
    with torch.inference_mode():
        output = model.generate(**inputs, max_new_tokens=max_tokens, do_sample=False)
    return processor.batch_decode(output[:, inputs["input_ids"].shape[1]:], skip_special_tokens=True)[0], time.perf_counter() - start


def exact_simple_id(raw, labels):
    """No recovery: only an exact stripped model answer is considered compliant."""
    answer = raw.strip()
    return answer if answer in labels | {"NONE"} else None


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
    started = time.perf_counter()
    processor = AutoProcessor.from_pretrained(args.model_dir, local_files_only=True)
    model = AutoModelForImageTextToText.from_pretrained(args.model_dir, dtype=torch.float16 if device == "cuda" else torch.float32, local_files_only=True).to(device).eval()
    load_seconds = time.perf_counter() - started
    args.output_dir.mkdir(parents=True)
    overlays = args.output_dir / "overlays"
    overlays.mkdir()
    records, latencies = [], []
    for observation_id in config["observations"]:
        root = public / observation_id
        image = Image.open(root / "rgb.png").convert("RGB")
        depth = __import__("numpy").load(root / "depth_m_float32.npy")
        info = json.loads((root / "camera_info.json").read_text())
        transform = camera_transform(calibration[observation_id]["self_pose"], calibration[observation_id]["camera_mounting"])
        raw_candidates = candidate_targets(depth, info, transform, [dict(item, observation_id=observation_id) for item in detections[observation_id]])
        display_candidates = []
        candidate_map = {}
        for label, candidate in enumerate(raw_candidates):
            display = dict(candidate)
            display["candidate_id"] = str(label)
            display_candidates.append(display)
            candidate_map[str(label)] = candidate["candidate_id"]
        annotated = draw_candidates(image, display_candidates)
        annotated.save(overlays / f"{observation_id}_integer_candidates.png")
        strict_raw, strict_latency = generate(model, processor, annotated, config["strict_json_prompt"], device, config["model"]["max_new_tokens"])
        simple_raw, simple_latency = generate(model, processor, annotated, config["simple_id_prompt"], device, config["model"]["max_new_tokens"])
        description_raw, description_latency = generate(model, processor, image, config["description_prompt"], device, config["model"]["max_new_tokens"])
        latencies.extend([strict_latency, simple_latency, description_latency])
        strict_parsed, strict_error = parse_selection(strict_raw, set(candidate_map) | {"NONE"})
        simple_label = exact_simple_id(simple_raw, set(candidate_map))
        records.append({
            "observation_id": observation_id,
            "candidate_label_to_opaque_id": candidate_map,
            "processor_input": {
                "strict_annotated": inspect_input(processor, annotated, config["strict_json_prompt"]),
                "simple_annotated": inspect_input(processor, annotated, config["simple_id_prompt"]),
                "description_plain_rgb": inspect_input(processor, image, config["description_prompt"])
            },
            "raw_outputs": {"strict_json": strict_raw, "simple_id": simple_raw, "plain_description": description_raw},
            "format_checks": {
                "strict_json_valid": strict_error is None,
                "strict_json_error": strict_error,
                "strict_candidate_label": strict_parsed.get("candidate_id") if strict_parsed else None,
                "simple_exact_valid": simple_label is not None,
                "simple_exact_label": simple_label,
                "simple_opaque_id": candidate_map.get(simple_label) if simple_label else None
            },
            "latency_seconds": {"strict_json": strict_latency, "simple_id": simple_latency, "plain_description": description_latency}
        })
    result = {
        "scope": "DEV_ONLY_FORMAT_COMPARISON_NO_ACTION_EXECUTION",
        "v1_preserved_at": "artifacts/companion-home-validation/evidence/vlm-grid-dev-v1/results/results.json",
        "model": {"id": config["model"]["id"], "revision": config["model"]["revision"]},
        "runtime": {"device": device, "load_seconds": load_seconds, "peak_cuda_allocated_mib": torch.cuda.max_memory_allocated() / 2**20 if device == "cuda" else None, "per_query_latency_seconds": latencies},
        "prompts": {key: config[key] for key in ("strict_json_prompt", "simple_id_prompt", "description_prompt")},
        "records": records,
        "summary": {"observations": len(records), "strict_json_valid": sum(item["format_checks"]["strict_json_valid"] for item in records), "simple_exact_valid": sum(item["format_checks"]["simple_exact_valid"] for item in records), "plain_nonempty_descriptions": sum(bool(item["raw_outputs"]["plain_description"].strip()) for item in records)},
        "replay": "python scripts/run_vlm_grid_format_v2.py --public-dir artifacts/companion-home-validation/evidence/vlm-grid-dev-v1/public-input --config config/vlm_grid_dev_v2.json --model-dir /home/gxxl/.cache/soarm-vlm-grid/models/smolvlm-500m --output-dir <new-output-dir>"
    }
    (args.output_dir / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result["summary"] | {"load_seconds": load_seconds, "peak_mib": result["runtime"]["peak_cuda_allocated_mib"]}))


if __name__ == "__main__":
    main()
