#!/usr/bin/env python3
"""Generate one predeclared held-out AWS-house layout without launching Gazebo."""
import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pose_values(text):
    values = [float(value) for value in text.split()]
    if len(values) != 6:
        raise ValueError("expected six-value model pose")
    return values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-world", type=Path, required=True)
    parser.add_argument("--layouts", type=Path, required=True)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise SystemExit("refusing to overwrite held-out layout output")
    plan = json.loads(args.layouts.read_text())
    lock = plan["source_asset_lock"]
    if digest(args.source_world) != lock["world_sha256"]:
        raise SystemExit("source world hash does not match held-out asset lock")
    layout = next((item for item in plan["layouts"] if item["scene_id"] == args.scene_id), None)
    if layout is None:
        raise SystemExit("unknown held-out scene id")
    tree = ET.parse(args.source_world)
    world = tree.getroot().find("world")
    models = {model.get("name"): model for model in world.findall("model")}
    changed = []
    for transform in layout["transforms"]:
        model = models.get(transform["name"])
        if model is None or model.findtext("pose") is None:
            raise SystemExit(f"missing named model pose: {transform['name']}")
        before = pose_values(model.findtext("pose"))
        radius = math.hypot(before[0], before[1])
        if radius == 0:
            raise SystemExit("cannot compute radial transform at world origin")
        radial = (before[0] / radius, before[1] / radius)
        tangent = (-radial[1], radial[0])
        after = list(before)
        after[0] += transform["radial_shift_m"] * radial[0] + transform["tangential_shift_m"] * tangent[0]
        after[1] += transform["radial_shift_m"] * radial[1] + transform["tangential_shift_m"] * tangent[1]
        model.find("pose").text = " ".join(f"{value:.9f}" for value in after)
        changed.append({"name": transform["name"], "before_pose": before, "after_pose": after, **transform})
    checks = plan["geometry_checks"]
    xmin, xmax, ymin, ymax = checks["world_xy_bounds_m"]
    for item in changed:
        x, y = item["after_pose"][:2]
        if not xmin <= x <= xmax or not ymin <= y <= ymax:
            raise SystemExit("changed chair leaves declared world bounds")
    minimum = 2 * checks["chair_radius_m"] + checks["minimum_changed_pair_clearance_m"]
    for index, first in enumerate(changed):
        for second in changed[index + 1:]:
            if math.dist(first["after_pose"][:2], second["after_pose"][:2]) < minimum:
                raise SystemExit("changed-chair footprint separation check failed")
    args.output_dir.mkdir(parents=True)
    destination = args.output_dir / "world.sdf"
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    (args.output_dir / "layout-lock.json").write_text(json.dumps({"scope": "PRIVATE_HELDOUT_LAYOUT_DESIGN_ONLY", "scene_id": args.scene_id, "seed": layout["seed"], "source_world_sha256": digest(args.source_world), "generated_world_sha256": digest(destination), "changed_transforms": changed, "geometry_checks": checks, "limitation": checks["limitation"]}, indent=2) + "\n")
    print(json.dumps({"scene_id": args.scene_id, "changed_models": len(changed), "generated_world_sha256": digest(destination)}))


if __name__ == "__main__":
    main()
