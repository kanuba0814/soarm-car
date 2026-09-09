#!/usr/bin/env python3
"""Prepare and verify locked position-only NavigateToPose-compatible map goals."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any
from xml.etree import ElementTree

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
CATALOG_PATH = REPOSITORY_ROOT / "artifacts" / "rooms-complete-reference.json"
LOCK_PATH = PACKAGE_ROOT / "asset_identity_lock.json"


class RequestError(ValueError):
    """A request or prepared payload is not safe for the existing consumer."""


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
        fail(f"{name} must be a finite non-boolean number")
    return float(value)


def load_json(path: Path, label: str) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        fail(f"cannot load {label}: {error}")


def load_lock() -> dict[str, Any]:
    return require_object(load_json(LOCK_PATH, "pinned asset identity lock"), "pinned asset identity lock")


def load_catalog(lock: dict[str, Any]) -> dict[str, dict[str, Any]]:
    if sha256(CATALOG_PATH) != require_string(lock.get("catalog_sha256"), "catalog_sha256"):
        fail("pinned catalog identity hash changed")
    catalog = require_object(load_json(CATALOG_PATH, "pinned catalog"), "pinned catalog")
    rooms = catalog.get("rooms")
    if not isinstance(rooms, list):
        fail("catalog.rooms must be a list")
    indexed: dict[str, dict[str, Any]] = {}
    for room in rooms:
        room = require_object(room, "catalog room")
        room_id = require_string(room.get("id"), "catalog room id")
        pose = room.get("goal_pose")
        if not isinstance(pose, list) or len(pose) != 3:
            fail(f"catalog goal_pose for {room_id} must contain x, y, yaw")
        if room_id in indexed:
            fail(f"duplicate catalog room id: {room_id}")
        for component, value in zip(("x", "y", "yaw"), pose):
            require_finite_number(value, f"catalog goal_pose.{component} for {room_id}")
        indexed[room_id] = room
    return indexed


def validate_assets(lock: dict[str, Any], asset_root: Path) -> None:
    assets = require_object(lock.get("assets"), "lock assets")
    for name, expected in assets.items():
        expected = require_object(expected, f"lock assets.{name}")
        relative_path = require_string(expected.get("path"), f"lock assets.{name}.path")
        path = asset_root / relative_path
        if not path.is_file():
            fail(f"locked asset missing: {relative_path}")
        actual_hash = sha256(path)
        compatible = expected.get("compatible_sha256")
        if compatible is not None:
            if name != "house_fastlio_nav2_launch" or not isinstance(compatible, list) or compatible != [
                    "c88daf45e9627d79fc13812b27795f57dd9936ebb39b40ba475aba3bbb7a3273",
                    "a520fa54f04feb26b2ceb8da8723046b89c2c8ab1d4cb1444cba63df17ec94d5"]:
                fail("invalid trusted launch compatibility lock")
            if actual_hash not in compatible:
                fail(f"locked asset changed: {relative_path}")
        elif actual_hash != require_string(expected.get("sha256"), f"lock assets.{name}.sha256"):
            fail(f"locked asset changed: {relative_path}")

    config_path = asset_root / assets["house_nav2_mppi_yaml"]["path"]
    try:
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        parameters = config["controller_server"]["ros__parameters"]
        checker = parameters["position_only_goal_checker"]
        if parameters["goal_checker_plugins"] != ["goal_checker", "position_only_goal_checker"]:
            fail("locked Nav2 YAML position-only checker list changed")
        if checker != {
            "plugin": "nav2_controller::SimpleGoalChecker",
            "xy_goal_tolerance": 0.2,
            "yaw_goal_tolerance": math.pi,
            "stateful": False,
        }:
            fail("locked Nav2 YAML position-only checker semantics changed")
    except (KeyError, TypeError, yaml.YAMLError) as error:
        fail(f"cannot interpret locked Nav2 YAML: {error}")

    position_bt = asset_root / assets["navigate_position_only_holonomic_bt"]["path"]
    through_bt = asset_root / assets["navigate_through_poses_holonomic_bt"]["path"]
    try:
        position_follow = ElementTree.parse(position_bt).find(".//FollowPath")
        through_follow = ElementTree.parse(through_bt).find(".//FollowPath")
    except (OSError, ElementTree.ParseError) as error:
        fail(f"cannot interpret locked behavior tree: {error}")
    if (position_follow is None or through_follow is None or
            position_follow.get("goal_checker_id") != "position_only_goal_checker" or
            through_follow.get("goal_checker_id") != "position_only_goal_checker"):
        fail("locked behavior tree does not select position_only_goal_checker")
    launch_text = (asset_root / assets["house_fastlio_nav2_launch"]["path"]).read_text(encoding="utf-8")
    if "navigate_position_only_holonomic.xml" not in launch_text:
        fail("locked launch does not select navigate_position_only_holonomic.xml")


def validate_request(request: Any, allowed_ids: list[str]) -> dict[str, Any]:
    request = require_object(request, "request")
    if set(request) != {"place_id", "frame_id", "require_heading"}:
        fail("request fields must be exactly place_id, frame_id, require_heading")
    place_id = require_string(request["place_id"], "place_id")
    if place_id not in allowed_ids:
        fail(f"unsupported place_id: {place_id}")
    if require_string(request["frame_id"], "frame_id") != "map":
        fail("frame_id must be locked map; no transform is available for foreign frames")
    if require_bool(request["require_heading"], "require_heading"):
        fail("unsupported_heading: active navigation is position-only")
    return {"place_id": place_id, "frame_id": "map", "require_heading": False}


def expected_goal(room: dict[str, Any]) -> dict[str, Any]:
    x, y, yaw = (require_finite_number(value, "catalog coordinate") for value in room["goal_pose"])
    return {
        "frame_id": "map",
        "position": {"x": x, "y": y, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)},
    }


def prepare(request_path: Path, asset_root: Path) -> dict[str, Any]:
    lock = load_lock()
    allowed_ids = lock.get("allowed_place_ids")
    if allowed_ids != ["room-1", "room-2", "room-3", "mian"]:
        fail("pinned place allowlist changed")
    validate_assets(lock, asset_root)
    request = validate_request(load_json(request_path, "request"), allowed_ids)
    room = load_catalog(lock).get(request["place_id"])
    if room is None:
        fail(f"allowed place absent from catalog: {request['place_id']}")
    return {
        "status": "prepared_for_serialization",
        "place_id": request["place_id"],
        "require_heading": False,
        "map_goal": expected_goal(room),
        "asset_manifest_sha256": sha256(LOCK_PATH),
    }


def verify_prepared(prepared: Any, asset_root: Path) -> dict[str, Any]:
    """Revalidate a persisted payload immediately before ROS goal construction."""
    lock = load_lock()
    if sha256(LOCK_PATH) != require_string(
            require_object(prepared, "prepared payload").get("asset_manifest_sha256"),
            "asset_manifest_sha256"):
        fail("prepared payload asset manifest hash does not match pinned manifest")
    if set(prepared) != {"status", "place_id", "require_heading", "map_goal", "asset_manifest_sha256"}:
        fail("prepared payload has unsupported fields")
    if prepared["status"] != "prepared_for_serialization":
        fail("prepared payload status is not prepared_for_serialization")
    allowed_ids = lock.get("allowed_place_ids")
    if not isinstance(allowed_ids, list):
        fail("pinned place allowlist invalid")
    request = validate_request({
        "place_id": prepared.get("place_id"),
        "frame_id": require_object(prepared.get("map_goal"), "map_goal").get("frame_id"),
        "require_heading": prepared.get("require_heading"),
    }, allowed_ids)
    validate_assets(lock, asset_root)
    room = load_catalog(lock).get(request["place_id"])
    if room is None:
        fail(f"allowed place absent from catalog: {request['place_id']}")
    actual_goal = require_object(prepared["map_goal"], "map_goal")
    if set(actual_goal) != {"frame_id", "position", "orientation"}:
        fail("map_goal fields changed")
    position = require_object(actual_goal["position"], "map_goal.position")
    orientation = require_object(actual_goal["orientation"], "map_goal.orientation")
    if set(position) != {"x", "y", "z"} or set(orientation) != {"x", "y", "z", "w"}:
        fail("map_goal coordinate fields changed")
    for name, value in position.items():
        require_finite_number(value, f"map_goal.position.{name}")
    for name, value in orientation.items():
        require_finite_number(value, f"map_goal.orientation.{name}")
    norm = sum(require_finite_number(orientation[key], f"map_goal.orientation.{key}") ** 2
               for key in ("x", "y", "z", "w"))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-12):
        fail("map_goal orientation is not a unit quaternion")
    if actual_goal != expected_goal(room):
        fail("prepared payload map_goal does not exactly match catalog pose")
    return actual_goal


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--request", type=Path)
    group.add_argument("--verify-prepared", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--asset-root", type=Path, default=REPOSITORY_ROOT)
    arguments = parser.parse_args()
    try:
        if arguments.request:
            if arguments.output is None:
                parser.error("--output is required with --request")
            result = prepare(arguments.request, arguments.asset_root)
            arguments.output.parent.mkdir(parents=True, exist_ok=True)
            arguments.output.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
        else:
            result = verify_prepared(load_json(arguments.verify_prepared, "prepared payload"), arguments.asset_root)
            print(json.dumps(result, sort_keys=True, allow_nan=False))
    except RequestError as error:
        print(json.dumps({"status": "not_ready", "error": str(error)}, sort_keys=True), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
