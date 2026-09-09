#!/usr/bin/env python3
"""Run a bounded DEV-only crop/context presentation comparison on local SmolVLM."""
import argparse
import hashlib
import json
import time
from pathlib import Path

import torch
from PIL import Image
from transformers import AutoModelForImageTextToText, AutoProcessor

from replay_vlm_label_adapter import adapt_label
from vlm_crop_context.prepare import prepare_views, rgb_content_sha256


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def image_record(image):
    image = image.convert("RGB")
    return {"mode": image.mode, "size_px": list(image.size), "rgb_content_sha256": rgb_content_sha256(image)}


def processor_inputs(processor, images, prompt):
    content = [{"type": "image"} for _ in images] + [{"type": "text", "text": prompt}]
    template = processor.apply_chat_template([{"role": "user", "content": content}], add_generation_prompt=True)
    inputs = processor(text=template, images=images, return_tensors="pt")
    return template, inputs


def generate(model, processor, images, prompt, device, max_tokens):
    template, inputs = processor_inputs(processor, images, prompt)
    inputs = inputs.to(device)
    started = time.perf_counter()
    with torch.inference_mode():
        output = model.generate(**inputs, max_new_tokens=max_tokens, do_sample=False)
    raw = processor.batch_decode(output[:, inputs["input_ids"].shape[1]:], skip_special_tokens=True)[0]
    return raw, time.perf_counter() - started, {
        "chat_template_sha256": hashlib.sha256(template.encode("utf-8")).hexdigest(),
        "tensor_shapes": {key: list(value.shape) for key, value in inputs.items() if hasattr(value, "shape")},
        "input_token_count": int(inputs["input_ids"].shape[-1]),
        "image_tile_count": int(inputs["pixel_values"].shape[1]) if "pixel_values" in inputs else None,
    }


def selection(raw, label_to_id):
    strict = raw.strip() if raw.strip() in set(label_to_id) | {"NONE"} else None
    adapted = adapt_label(raw, label_to_id)
    label = adapted if adapted is not None else strict
    return {
        "raw": raw,
        "strict_format_valid": strict is not None,
        "finite_label_adapter_valid": adapted is not None,
        "adapter_label": adapted,
        "no_match": label == "NONE",
        "selected_id": label_to_id.get(label),
        "semantic_correctness": "NOT_EVALUATED",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--public-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise SystemExit("refusing to overwrite output directory")
    config = json.loads(args.config.read_text())
    public = args.public_dir
    detections = json.loads((public / "detections.json").read_text())["observations"]
    if len(config["observations"]) != 4:
        raise SystemExit("this bounded experiment requires exactly four observations")
    calls_per_view = len(config["presentation_modes"]) + 1
    if calls_per_view * len(config["observations"]) > 24:
        raise SystemExit("generation-call budget exceeded")
    device = "cuda" if torch.cuda.is_available() else "cpu"
    if device != "cuda":
        raise SystemExit("GPU inference is required for this experiment")
    torch.cuda.reset_peak_memory_stats()
    started = time.perf_counter()
    processor = AutoProcessor.from_pretrained(args.model_dir, local_files_only=True)
    model = AutoModelForImageTextToText.from_pretrained(
        args.model_dir, dtype=torch.float16, local_files_only=True
    ).to(device).eval()
    load_seconds = time.perf_counter() - started
    args.output_dir.mkdir(parents=True)
    overlays = args.output_dir / "overlays"
    overlays.mkdir()
    records, calls = [], []
    for observation_id in config["observations"]:
        image = Image.open(public / observation_id / "rgb.png").convert("RGB")
        frame_detections = [dict(item, observation_id=observation_id) for item in detections[observation_id]]
        labels = [str(index) for index in range(len(frame_detections))]
        prepared = prepare_views(image, frame_detections, labels)
        for mode, rendered in (("full_annotated", prepared["full_annotated"]), ("crop_contact_sheet", prepared["crop_contact_sheet"]), ("full_plus_crop_panels", prepared["full_plus_crop_panels"])):
            rendered.save(overlays / f"{observation_id}_{mode}.png")
        mapping = {candidate["display_label"]: candidate["opaque_id"] for candidate in prepared["candidates"]}
        mode_images = {
            "full_annotated": [prepared["full_annotated"]],
            "crop_contact_sheet": [prepared["crop_contact_sheet"]],
            "full_plus_crop_context": [prepared["full_annotated"], prepared["crop_contact_sheet"]],
        }
        outputs = {}
        for mode in config["presentation_modes"]:
            raw, latency, processor_record = generate(model, processor, mode_images[mode], config["candidate_prompt"], device, config["model"]["max_new_tokens"])
            outputs[mode] = selection(raw, mapping) | {"latency_seconds": latency, "processor_input": processor_record, "images": [image_record(item) for item in mode_images[mode]]}
            calls.append(latency)
        permuted_labels = list(reversed(labels))
        permuted = prepare_views(image, frame_detections, permuted_labels)
        permuted["full_annotated"].save(overlays / f"{observation_id}_full_annotated_permuted.png")
        permuted_mapping = {candidate["display_label"]: candidate["opaque_id"] for candidate in permuted["candidates"]}
        raw, latency, processor_record = generate(model, processor, [permuted["full_annotated"]], config["candidate_prompt"], device, config["model"]["max_new_tokens"])
        outputs["full_annotated_reversed_labels"] = selection(raw, permuted_mapping) | {"latency_seconds": latency, "processor_input": processor_record, "images": [image_record(permuted["full_annotated"])]}
        calls.append(latency)
        records.append({"observation_id": observation_id, "source_image": image_record(image), "candidate_mapping": mapping, "candidates": prepared["candidates"], "outputs": outputs})
    if len(calls) > 24:
        raise RuntimeError("generation-call budget exceeded")
    results = {
        "scope": "DEV_ONLY_NO_SEMANTIC_ACCURACY_OR_ACTION_CLAIM",
        "inputs": {"manifest_sha256": digest(public / "manifest.json"), "detections_sha256": digest(public / "detections.json"), "config_sha256": digest(args.config)},
        "model": {"id": config["model"]["id"], "revision": config["model"]["revision"], "model_dir": str(args.model_dir)},
        "fixed_prompt": config["candidate_prompt"],
        "context_pixel_budget_note": config["context_transport"],
        "runtime": {"device": device, "cold_model_load_seconds": load_seconds, "first_generation_seconds": calls[0], "warm_generation_seconds": calls[1:], "generation_calls": len(calls), "max_new_tokens": config["model"]["max_new_tokens"], "peak_cuda_allocated_mib": torch.cuda.max_memory_allocated() / 2**20},
        "label_permutation_note": config["label_permutation"],
        "records": records,
        "summary": {"observations": len(records), "strict_format_valid": sum(output["strict_format_valid"] for record in records for output in record["outputs"].values()), "finite_label_adapter_valid": sum(output["finite_label_adapter_valid"] for record in records for output in record["outputs"].values()), "calls": len(calls), "semantic_accuracy": "NOT_EVALUATED"},
        "replay": "python scripts/run_vlm_crop_context_v1.py --public-dir artifacts/companion-home-validation/evidence/vlm-grid-dev-v1/public-input --config config/vlm_crop_context_v1.json --model-dir /home/gxxl/.cache/soarm-vlm-grid/models/smolvlm-500m --output-dir <new-output-dir>",
    }
    (args.output_dir / "results.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results["summary"] | {"load_seconds": load_seconds, "peak_mib": results["runtime"]["peak_cuda_allocated_mib"]}))


if __name__ == "__main__":
    main()
