#!/usr/bin/env python3
"""Exercise the prepared JSON through the installed NavigateToPose message type."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
PACKAGE_ROOT = REPOSITORY_ROOT / "ros2_ws/src/soarm_named_place_request"
MODULE_ROOT = PACKAGE_ROOT
CATALOG = REPOSITORY_ROOT / "artifacts/rooms-complete-reference.json"
LOCK = PACKAGE_ROOT / "asset_identity_lock.json"
BASELINE = Path("/home/gxxl/soarm-car-nav2-clean")
IMAGE = "soarm-car-nav2-ros2-dev:latest"


def command(arguments: list[str], expected_returncode: int = 0) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(arguments, text=True, capture_output=True, check=False)
    if result.returncode != expected_returncode:
        raise AssertionError(
            f"expected {expected_returncode}, got {result.returncode}: {arguments}\n"
            f"stdout={result.stdout}\nstderr={result.stderr}"
        )
    return result


def copy_locked_assets(asset_root: Path) -> None:
    paths = [
        "ros2_ws/src/soarm_simulation/maps/house_floorplan.pgm",
        "ros2_ws/src/soarm_navigation/soarm_navigation/launch/house_fastlio_nav2.launch.py",
        "ros2_ws/src/soarm_navigation/config/house_nav2_mppi.yaml",
        "ros2_ws/src/soarm_navigation/behavior_trees/navigate_holonomic.xml",
        "ros2_ws/src/soarm_navigation/behavior_trees/navigate_through_poses_holonomic.xml",
    ]
    for relative_path in paths:
        source = BASELINE / relative_path if relative_path.endswith("house_floorplan.pgm") else REPOSITORY_ROOT / relative_path
        destination = asset_root / relative_path
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
    output = asset_root / "artifacts/house_widened_reference.pcd"
    command([
        "python3", str(BASELINE / "scripts/generate_house_reference_pcd.py"),
        "--output", str(output),
        "--map-output", str(asset_root / "unused-generated-map.pgm"),
    ])


def run_cli(request: dict, output: Path, asset_root: Path, catalog: Path = CATALOG, lock: Path = LOCK) -> subprocess.CompletedProcess[str]:
    request_path = output.with_suffix(".request.json")
    request_path.write_text(json.dumps(request, allow_nan=False), encoding="utf-8")
    return command([
        "python3", "-m", "soarm_named_place_request.prepare",
        "--request", str(request_path), "--output", str(output),
        "--catalog", str(catalog), "--lock", str(lock), "--asset-root", str(asset_root),
    ])


def assert_rejected(request: dict, output: Path, asset_root: Path, expected_error: str, catalog: Path = CATALOG, lock: Path = LOCK) -> None:
    request_path = output.with_suffix(".request.json")
    request_path.write_text(json.dumps(request, allow_nan=False), encoding="utf-8")
    result = command([
        "python3", "-m", "soarm_named_place_request.prepare",
        "--request", str(request_path), "--output", str(output),
        "--catalog", str(catalog), "--lock", str(lock), "--asset-root", str(asset_root),
    ], expected_returncode=2)
    assert expected_error in result.stderr, result.stderr
    assert not output.exists()


def verify_ros_consumer(output_paths: list[Path]) -> None:
    inspect_script = """
import json
import math
import sys
from nav2_msgs.action import NavigateToPose
from rclpy.serialization import deserialize_message, serialize_message
for filename in sys.argv[1:]:
    prepared = json.load(open(filename, encoding='utf-8'))
    assert prepared['status'] == 'prepared_for_serialization'
    assert prepared['require_heading'] is True
    goal_data = prepared['map_goal']
    assert goal_data['frame_id'] == 'map'
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = goal_data['frame_id']
    goal.pose.pose.position.x = goal_data['position']['x']
    goal.pose.pose.position.y = goal_data['position']['y']
    goal.pose.pose.position.z = goal_data['position']['z']
    goal.pose.pose.orientation.x = goal_data['orientation']['x']
    goal.pose.pose.orientation.y = goal_data['orientation']['y']
    goal.pose.pose.orientation.z = goal_data['orientation']['z']
    goal.pose.pose.orientation.w = goal_data['orientation']['w']
    replayed = deserialize_message(serialize_message(goal), NavigateToPose.Goal)
    assert replayed.pose.header.frame_id == 'map'
    assert math.isclose(replayed.pose.pose.position.x, goal_data['position']['x'])
    assert math.isclose(replayed.pose.pose.position.y, goal_data['position']['y'])
    assert math.isclose(replayed.pose.pose.position.z, 0.0)
    assert math.isclose(replayed.pose.pose.orientation.z, goal_data['orientation']['z'])
    assert math.isclose(replayed.pose.pose.orientation.w, goal_data['orientation']['w'])
print('NavigateToPose.Goal serialize/deserialize passed for', len(sys.argv) - 1, 'prepared requests')
"""
    inspect_path = output_paths[0].parent / "inspect_navigate_to_pose.py"
    inspect_path.write_text(inspect_script, encoding="utf-8")
    command([
        "docker", "run", "--rm", "--network", "none",
        "-v", f"{REPOSITORY_ROOT}:{REPOSITORY_ROOT}:ro",
        "-v", "/tmp:/tmp:ro",
        "--entrypoint", "bash", IMAGE, "-lc",
        "source /opt/ros/humble/setup.bash && python3 " + str(inspect_path) + " " + " ".join(map(str, output_paths)),
    ])


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="soarm-nav-request-") as temporary:
        temporary_root = Path(temporary)
        asset_root = temporary_root / "assets"
        copy_locked_assets(asset_root)
        outputs = []
        catalog = json.loads(CATALOG.read_text(encoding="utf-8"))
        expected = {room["id"]: room["goal_pose"] for room in catalog["rooms"]}
        for place_id in ("room-1", "room-2", "room-3", "mian"):
            output = temporary_root / f"{place_id}.json"
            run_cli({"place_id": place_id, "frame_id": "map", "require_heading": True}, output, asset_root)
            prepared = json.loads(output.read_text(encoding="utf-8"))
            x, y, yaw = expected[place_id]
            assert prepared["map_goal"]["position"] == {"x": float(x), "y": float(y), "z": 0.0}
            assert math.isclose(prepared["map_goal"]["orientation"]["z"], math.sin(yaw / 2.0))
            assert math.isclose(prepared["map_goal"]["orientation"]["w"], math.cos(yaw / 2.0))
            outputs.append(output)
        verify_ros_consumer(outputs)

        rejected = temporary_root / "rejected.json"
        valid = {"place_id": "room-1", "frame_id": "map", "require_heading": True}
        assert_rejected({**valid, "place_id": "unknown"}, rejected, asset_root, "unknown place_id")
        assert_rejected({**valid, "frame_id": "camera_rgb_optical_frame"}, rejected, asset_root, "no transform is available")
        assert_rejected({**valid, "require_heading": False}, rejected, asset_root, "require_heading must be true")
        assert_rejected({"place_id": "room-1", "require_heading": True}, rejected, asset_root, "request fields")

        pgm = asset_root / "ros2_ws/src/soarm_simulation/maps/house_floorplan.pgm"
        pgm.write_bytes(pgm.read_bytes() + b"changed")
        assert_rejected(valid, rejected, asset_root, "locked asset changed")
        copy_locked_assets(asset_root)
        (asset_root / "artifacts/house_widened_reference.pcd").unlink()
        assert_rejected(valid, rejected, asset_root, "locked asset missing")
        copy_locked_assets(asset_root)

        changed_catalog = temporary_root / "changed-catalog.json"
        changed_catalog.write_text(CATALOG.read_text(encoding="utf-8") + " ", encoding="utf-8")
        assert_rejected(valid, rejected, asset_root, "catalog identity hash changed", changed_catalog)

        nonfinite_catalog = temporary_root / "nonfinite-catalog.json"
        nonfinite = json.loads(CATALOG.read_text(encoding="utf-8"))
        nonfinite["rooms"][2]["goal_pose"][0] = float("nan")
        nonfinite_catalog.write_text(json.dumps(nonfinite, allow_nan=True), encoding="utf-8")
        nonfinite_lock = temporary_root / "nonfinite-lock.json"
        lock = json.loads(LOCK.read_text(encoding="utf-8"))
        lock["catalog_sha256"] = hashlib.sha256(nonfinite_catalog.read_bytes()).hexdigest()
        nonfinite_lock.write_text(json.dumps(lock), encoding="utf-8")
        assert_rejected(valid, rejected, asset_root, "must be a finite number", nonfinite_catalog, nonfinite_lock)


if __name__ == "__main__":
    main()
    print("named-place request consumer compatibility tests passed")
