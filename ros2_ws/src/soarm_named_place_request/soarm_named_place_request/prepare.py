#!/usr/bin/env python3
"""Prepare, but never send, a locked NavigateToPose-compatible map goal."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
DEFAULT_CATALOG = REPOSITORY_ROOT / "artifacts" / "rooms-complete-reference.json"
DEFAULT_LOCK = PACKAGE_ROOT / "asset_identity_lock.json"


class RequestError(ValueError):
    """A request is not safe to prepare for serialization."""


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as input_file:
        for chunk in iter(lambda: input_file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def fail(message: str) -> None:
    raise RequestError(message)


def require_object(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        fail(f"{name} must be an object")
    return value


def require_string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value:
        fail(f"{name} must be a non-empty string")
    return value


def require_bool(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        fail(f"{name} must be a boolean")
    return value


def require_finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        fail(f"{name} must be a finite number")
    return float(value)


def load_json(path: Path, label: str) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        fail(f"cannot load {label}: {error}")


def validate_request(request: Any) -> dict[str, Any]:
    request = require_object(request, "request")
    if set(request) != {"place_id", "frame_id", "require_heading"}:
        fail("request fields must be exactly place_id, frame_id, require_heading")
    place_id = require_string(request["place_id"], "place_id")
    frame_id = require_string(request["frame_id"], "frame_id")
    require_heading = require_bool(request["require_heading"], "require_heading")
    if frame_id != "map":
        fail("frame_id must be locked map; no transform is available for foreign frames")
    if not require_heading:
        fail("require_heading must be true; position-only navigation cannot enforce terminal heading")
    return {"place_id": place_id, "frame_id": frame_id, "require_heading": require_heading}


def validate_catalog(catalog: Any) -> dict[str, dict[str, Any]]:
    catalog = require_object(catalog, "catalog")
    rooms = catalog.get("rooms")
    if not isinstance(rooms, list):
        fail("catalog.rooms must be a list")
    indexed = {}
    for room in rooms:
        room = require_object(room, "catalog room")
        room_id = require_string(room.get("id"), "catalog room id")
        pose = room.get("goal_pose")
        if not isinstance(pose, list) or len(pose) != 3:
            fail(f"catalog goal_pose for {room_id} must contain x, y, yaw")
        if room_id in indexed:
            fail(f"duplicate catalog room id: {room_id}")
        indexed[room_id] = room
        for component, value in zip(("x", "y", "yaw"), pose):
            require_finite_number(value, f"catalog goal_pose.{component} for {room_id}")
    return indexed


def validate_assets(lock: dict[str, Any], asset_root: Path, catalog_path: Path) -> dict[str, Any]:
    expected_catalog_hash = require_string(lock.get("catalog_sha256"), "lock catalog_sha256")
    if sha256(catalog_path) != expected_catalog_hash:
        fail("catalog identity hash changed")
    assets = require_object(lock.get("assets"), "lock assets")
    observed = {}
    for name, expected in assets.items():
        expected = require_object(expected, f"lock assets.{name}")
        relative_path = require_string(expected.get("path"), f"lock assets.{name}.path")
        expected_hash = require_string(expected.get("sha256"), f"lock assets.{name}.sha256")
        path = asset_root / relative_path
        if not path.is_file():
            fail(f"locked asset missing: {relative_path}")
        actual_hash = sha256(path)
        if actual_hash != expected_hash:
            fail(f"locked asset changed: {relative_path}")
        observed[name] = {"path": relative_path, "sha256": actual_hash}

    nav_yaml_path = asset_root / require_string(lock.get("nav_yaml_path"), "lock nav_yaml_path")
    try:
        nav_config = yaml.safe_load(nav_yaml_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        fail(f"cannot interpret locked Nav2 YAML: {error}")
    try:
        parameters = nav_config["bt_navigator"]["ros__parameters"]
        if parameters["global_frame"] != "map":
            fail("locked Nav2 YAML global_frame is not map")
        if parameters["default_nav_to_pose_bt_xml"] != "navigate_holonomic.xml":
            fail("locked Nav2 YAML NavigateToPose tree reference changed")
    except (KeyError, TypeError) as error:
        fail(f"locked Nav2 YAML schema changed: {error}")
    return observed


def prepare(request_path: Path, catalog_path: Path, lock_path: Path, asset_root: Path) -> dict[str, Any]:
    request = validate_request(load_json(request_path, "request"))
    lock = require_object(load_json(lock_path, "asset identity lock"), "asset identity lock")
    observed_assets = validate_assets(lock, asset_root, catalog_path)
    rooms = validate_catalog(load_json(catalog_path, "catalog"))
    room = rooms.get(request["place_id"])
    if room is None:
        fail(f"unknown place_id: {request['place_id']}")
    goal_x, goal_y, goal_yaw = (
        require_finite_number(value, "catalog coordinate") for value in room["goal_pose"]
    )
    return {
        "status": "prepared_for_serialization",
        "place_id": request["place_id"],
        "require_heading": True,
        "map_goal": {
            "frame_id": "map",
            "position": {"x": goal_x, "y": goal_y, "z": 0.0},
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": math.sin(goal_yaw / 2.0),
                "w": math.cos(goal_yaw / 2.0),
            },
        },
        "asset_identity": {
            "catalog_sha256": lock["catalog_sha256"],
            "assets": observed_assets,
            "identity_scope": "asset identity only; not geometry, reachability, or physical safety certification",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--lock", type=Path, default=DEFAULT_LOCK)
    parser.add_argument("--asset-root", type=Path, default=REPOSITORY_ROOT)
    arguments = parser.parse_args()
    try:
        prepared = prepare(arguments.request, arguments.catalog, arguments.lock, arguments.asset_root)
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(
            json.dumps(prepared, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
    except RequestError as error:
        print(json.dumps({"status": "not_ready", "error": str(error)}, sort_keys=True), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
