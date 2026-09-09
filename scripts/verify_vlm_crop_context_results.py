#!/usr/bin/env python3
"""Read a saved crop/context result in a fresh process and verify candidate mappings."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--public-dir", type=Path, required=True)
    args = parser.parse_args()
    results = json.loads(args.results.read_text())
    detections = json.loads((args.public_dir / "detections.json").read_text())["observations"]
    if results["runtime"]["generation_calls"] > 24:
        raise SystemExit("generation call budget violated")
    for record in results["records"]:
        expected = {f"{record['observation_id']}:{item['detection_index']}" for item in detections[record["observation_id"]]}
        candidate_ids = {candidate["opaque_id"] for candidate in record["candidates"]}
        if candidate_ids != expected:
            raise SystemExit(f"candidate mapping mismatch for {record['observation_id']}")
        for candidate in record["candidates"]:
            if not candidate["source_rgb_sha256"] or candidate["clipped_box_xyxy"][2] <= candidate["clipped_box_xyxy"][0] or candidate["clipped_box_xyxy"][3] <= candidate["clipped_box_xyxy"][1]:
                raise SystemExit(f"invalid prepared candidate for {record['observation_id']}")
        for output in record["outputs"].values():
            if output["selected_id"] is not None and output["selected_id"] not in expected:
                raise SystemExit(f"selected ID outside supplied boxes for {record['observation_id']}")
    print(json.dumps({"verified_records": len(results["records"]), "generation_calls": results["runtime"]["generation_calls"], "semantic_accuracy": results["summary"]["semantic_accuracy"]}))


if __name__ == "__main__":
    main()
