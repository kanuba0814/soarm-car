#!/usr/bin/env python3
"""Write a resumable held-out capture/evaluation batch plan; never executes capture."""
import argparse
import hashlib
import json
from pathlib import Path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--layouts", type=Path, required=True)
    parser.add_argument("--a1-config", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    args = parser.parse_args()
    if args.output_root.exists():
        raise SystemExit("refusing to overwrite held-out batch root")
    layouts = json.loads(args.layouts.read_text())
    survey = Path(layouts["fixed_survey_pose_config"])
    batch = {"scope": "READY_FOR_CAPTURE_NOT_EXECUTED", "layout_plan_sha256": digest(args.layouts), "a1_config_sha256": digest(args.a1_config), "survey_pose_sha256": digest(survey), "planned_rows_total": layouts["planned_protocol"]["planned_rows_total"], "layouts": []}
    for layout in layouts["layouts"]:
        root = args.output_root / layout["scene_id"]
        batch["layouts"].append({"scene_id": layout["scene_id"], "seed": layout["seed"], "runtime_root": str(root), "refuse_if_exists": [str(root / name) for name in ("capture", "public-input", "method", "private-evaluation", "raw-archive")], "steps": ["generate locked world and evaluator runtime overlay", "capture eight fixed survey poses with at most two attempts per view and retain all failures", "archive raw capture", "sanitize broker", "bwrap public runner", "private evaluator", "aggregate per-layout paired rows and unknown counts"]})
    args.output_root.mkdir(parents=True)
    (args.output_root / "batch-plan.json").write_text(json.dumps(batch, indent=2) + "\n")
    print(json.dumps({"status": "READY_FOR_CAPTURE", "layouts": len(batch["layouts"]), "planned_rows": batch["planned_rows_total"]}))


if __name__ == "__main__":
    main()
