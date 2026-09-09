#!/usr/bin/env python3
"""Resumable held-out A1 batch executor; capture is release-gated."""
import argparse
import hashlib
import json
import os
import shlex
import subprocess
import sys
from pathlib import Path


REQUIRED_OUTPUTS = ("capture/capture.json", "yolo/results.json", "package/public/manifest.json", "package/private/instances.json", "public-input/detections.json", "public-method/method.json", "private-evaluation/evaluation.json")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_layouts(batch, layout_root):
    layouts = []
    for entry in batch["layouts"]:
        root = layout_root / entry["scene_id"]
        lock, world = root / "layout-lock.json", root / "world.sdf"
        if not lock.exists() or not world.exists():
            raise RuntimeError(f"missing generated layout {entry['scene_id']}")
        item = json.loads(lock.read_text())
        if item["generated_world_sha256"] != digest(world) or len(item["changed_transforms"]) < 2:
            raise RuntimeError(f"invalid layout lock {entry['scene_id']}")
        layouts.append({"scene_id": entry["scene_id"], "lock": lock, "world": world, "world_sha256": item["generated_world_sha256"]})
    return layouts


def renderer_environment(layout):
    return {"A1_WORLD_OVERRIDE": str(layout["world"].resolve()), "A1_WORLD_OVERRIDE_SHA256": layout["world_sha256"], "COMPANION_EVALUATOR_CAPTURE": "1"}


def commands(layout, runtime_root, args):
    root = runtime_root / layout["scene_id"]
    capture = root / "capture"
    yolo = root / "yolo"
    package = root / "package"
    public_input = root / "public-input"
    method = root / "public-output" / "method"
    private = root / "private-evaluation"
    python = sys.executable
    return [
        [python, "scripts/companion_capture.py", str(capture), args.survey_config],
        [python, "scripts/run_companion_yoloworld_smoke.py", "--input-dir", str(capture), "--output-dir", str(yolo), "--confidence", "0.10"],
        [python, "scripts/companion_evaluator_gt.py", "--input-dir", str(capture), "--yolo-results", str(yolo / "results.json"), "--output-dir", str(package)],
        [python, "scripts/prepare_a1_visualconf_public_inputs.py", "--public-dir", str(package / "public"), "--frozen-detections", str(yolo / "results.json"), "--acquisition-dir", str(capture), "--config", args.a1_config, "--output-dir", str(public_input)],
        ["bwrap", "--ro-bind", "/usr", "/usr", "--ro-bind", "/lib", "/lib", "--ro-bind", "/lib64", "/lib64", "--ro-bind", str(Path(sys.executable).parents[1]), str(Path(sys.executable).parents[1]), "--ro-bind", str(Path.cwd() / "scripts"), "/code", "--ro-bind", str(package / "public"), "/public", "--ro-bind", str(public_input), "/input", "--ro-bind", str(Path(args.a1_config).resolve()), "/config.json", "--proc", "/proc", "--dev", "/dev", "--bind", str(method.parent), "/method-parent", sys.executable, "/code/run_a1_visualconf_public.py", "--public-dir", "/public", "--public-input-dir", "/input", "--config", "/config.json", "--output-dir", "/method-parent/method"],
        [python, "scripts/evaluate_a1_visualconf_private.py", "--private-dir", str(package / "private"), "--public-dir", str(package / "public"), "--public-input-dir", str(public_input), "--method", str(method / "method.json"), "--config", args.a1_config, "--output-dir", str(private)],
    ]


def receipt(layout, runtime_root, args):
    root = runtime_root / layout["scene_id"]
    files = [root / item for item in REQUIRED_OUTPUTS]
    if not all(path.exists() for path in files):
        return None
    return {"scene_id": layout["scene_id"], "world_sha256": layout["world_sha256"], "a1_config_sha256": digest(args.a1_config), "outputs": {str(path.relative_to(root)): digest(path) for path in files}}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--batch-plan", type=Path, required=True)
    parser.add_argument("--layout-root", type=Path, required=True)
    parser.add_argument("--runtime-root", type=Path, required=True)
    parser.add_argument("--survey-config", required=True)
    parser.add_argument("--a1-config", required=True)
    parser.add_argument("--renderer-command", help="shell command that starts the renderer and honors A1_WORLD_OVERRIDE")
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if args.prepare == args.execute:
        raise SystemExit("choose exactly one of --prepare or --execute")
    batch = json.loads(args.batch_plan.read_text())
    layouts = load_layouts(batch, args.layout_root)
    constructed = {item["scene_id"]: commands(item, args.runtime_root, args) for item in layouts}
    if args.prepare:
        print(json.dumps({"status": "READY_FOR_CAPTURE_NOT_EXECUTED", "layouts": [{"scene_id": item["scene_id"], "world": str(item["world"]), "world_sha256": item["world_sha256"], "commands": constructed[item["scene_id"]]} for item in layouts], "planned_rows": batch["planned_rows_total"]}))
        return
    if os.environ.get("A1_RENDERER_RELEASED") != "1":
        raise SystemExit("--execute is disabled until parent sets A1_RENDERER_RELEASED=1")
    if not args.renderer_command:
        raise SystemExit("--execute requires --renderer-command")
    args.runtime_root.mkdir(parents=True, exist_ok=True)
    completed = []
    for layout in layouts:
        existing = receipt(layout, args.runtime_root, args)
        if existing is not None:
            completed.append(existing)
            continue
        root = args.runtime_root / layout["scene_id"]
        if root.exists():
            raise SystemExit(f"refusing partial or mismatched runtime directory {root}")
        root.mkdir()
        (root / "public-output").mkdir()
        environment = os.environ | renderer_environment(layout)
        renderer = subprocess.Popen(args.renderer_command, shell=True, env=environment)
        try:
            for command in constructed[layout["scene_id"]]:
                subprocess.run(command, check=True, env=environment)
        finally:
            renderer.terminate()
            renderer.wait(timeout=30)
        done = receipt(layout, args.runtime_root, args)
        if done is None:
            raise RuntimeError(f"incomplete execution receipt for {layout['scene_id']}")
        (root / "receipt.json").write_text(json.dumps(done, indent=2) + "\n")
        completed.append(done)
    (args.runtime_root / "aggregate-receipts.json").write_text(json.dumps({"status": "COMPLETED_ONLY", "layouts": completed}, indent=2) + "\n")
    print(json.dumps({"status": "COMPLETED", "layouts": len(completed)}))


if __name__ == "__main__":
    main()
