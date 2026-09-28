#!/usr/bin/env python3
"""Score a sequence-level object inventory from frozen human adjudication."""

import argparse
import json
from pathlib import Path


def load_json(path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ground_truth", type=Path)
    parser.add_argument("prediction", type=Path)
    parser.add_argument("adjudication", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    ground_truth = load_json(args.ground_truth)
    prediction = load_json(args.prediction)
    adjudication = load_json(args.adjudication)

    visible_ground_truth = {
        item["instance_id"]: item
        for item in ground_truth["instances"]
        if item.get("count_for_inventory", True)
    }
    predictions = {
        item["instance_id"]: item for item in prediction["instances"]
    }
    if len(predictions) != len(prediction["instances"]):
        raise ValueError("prediction instance_id values must be unique")

    accepted_matches = []
    matched_ground_truth = set()
    matched_predictions = set()
    for match in adjudication["matches"]:
        if not match.get("accepted", False):
            continue
        prediction_id = match["prediction_id"]
        ground_truth_id = match["ground_truth_id"]
        if prediction_id not in predictions:
            raise ValueError(f"unknown prediction_id: {prediction_id}")
        if ground_truth_id not in visible_ground_truth:
            raise ValueError(f"unknown or excluded ground_truth_id: {ground_truth_id}")
        if prediction_id in matched_predictions:
            raise ValueError(f"prediction matched more than once: {prediction_id}")
        if ground_truth_id in matched_ground_truth:
            raise ValueError(f"ground-truth instance matched more than once: {ground_truth_id}")
        matched_predictions.add(prediction_id)
        matched_ground_truth.add(ground_truth_id)
        accepted_matches.append(match)

    prediction_count = len(predictions)
    ground_truth_count = len(visible_ground_truth)
    correct_count = len(accepted_matches)
    duplicate_ids = set(adjudication.get("duplicate_prediction_ids", []))
    unknown_duplicate_ids = duplicate_ids - predictions.keys()
    if unknown_duplicate_ids:
        raise ValueError(f"unknown duplicate prediction IDs: {sorted(unknown_duplicate_ids)}")
    unnamed_count = sum(not item.get("canonical_name", "").strip() for item in predictions.values())

    score = {
        "schema": "soarm.inventory_score.v1",
        "sequence_id": ground_truth["sequence_id"],
        "system": prediction["system"],
        "counts": {
            "ground_truth_visible_instances": ground_truth_count,
            "predicted_instances": prediction_count,
            "correctly_matched_instances": correct_count,
            "duplicate_predictions": len(duplicate_ids),
            "unnamed_predictions": unnamed_count,
        },
        "metrics": {
            "inventory_recall": correct_count / ground_truth_count if ground_truth_count else None,
            "inventory_precision": correct_count / prediction_count if prediction_count else 0.0,
            "duplicate_rate": len(duplicate_ids) / prediction_count if prediction_count else 0.0,
            "unnamed_rate": unnamed_count / prediction_count if prediction_count else 0.0,
            "recall_target_met": bool(ground_truth_count and correct_count / ground_truth_count >= 0.8),
        },
    }
    rendered = json.dumps(score, indent=2, ensure_ascii=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)


if __name__ == "__main__":
    main()
