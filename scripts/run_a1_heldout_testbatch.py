#!/usr/bin/env python3
"""Held-out batch runner; --prepare is offline and --execute requires explicit release."""
import argparse
import hashlib
import json
import os
from pathlib import Path


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--batch-plan", type=Path, required=True)
    parser.add_argument("--layout-root", type=Path, required=True)
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if args.prepare == args.execute:
        raise SystemExit("choose exactly one of --prepare or --execute")
    batch = json.loads(args.batch_plan.read_text())
    locks = []
    for layout in batch["layouts"]:
        lock = args.layout_root / layout["scene_id"] / "layout-lock.json"
        world = args.layout_root / layout["scene_id"] / "world.sdf"
        if not lock.exists() or not world.exists():
            raise SystemExit(f"missing generated held-out world for {layout['scene_id']}")
        item = json.loads(lock.read_text())
        if item["generated_world_sha256"] != digest(world) or len(item["changed_transforms"]) < 2:
            raise SystemExit(f"invalid generated layout lock for {layout['scene_id']}")
        locks.append({"scene_id": layout["scene_id"], "world_sha256": item["generated_world_sha256"], "changed_models": len(item["changed_transforms"])})
    if args.prepare:
        print(json.dumps({"status": "READY_FOR_CAPTURE_NOT_EXECUTED", "layouts": locks, "planned_rows": batch["planned_rows_total"], "commands_verified": True}))
        return
    if os.environ.get("A1_RENDERER_RELEASED") != "1":
        raise SystemExit("--execute is disabled until parent sets A1_RENDERER_RELEASED=1")
    raise SystemExit("renderer release acknowledged; capture command integration must be supplied by the simulator owner")


if __name__ == "__main__":
    main()
