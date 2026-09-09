#!/usr/bin/env python3
"""Exercise locked position-only requests through the installed ROS message consumer."""

from __future__ import annotations

import json
import math
from pathlib import Path
import subprocess
import tempfile


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
PACKAGE_ROOT = REPOSITORY_ROOT / "ros2_ws/src/soarm_named_place_request"
MODULE_ROOT = PACKAGE_ROOT
BASELINE = Path("/home/gxxl/soarm-car-nav2-clean")
LOCK = PACKAGE_ROOT / "asset_identity_lock.json"
IMAGE_ID = "sha256:15012136e44c6a439c137ac62a67c0f38f09f55a357ba03a916a48895f6e4dc9"


def command(arguments: list[str], expected_returncode: int = 0) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(arguments, text=True, capture_output=True, check=False)
    if result.returncode != expected_returncode:
        raise AssertionError(
            f"expected {expected_returncode}, got {result.returncode}: {arguments}\n"
            f"stdout={result.stdout}\nstderr={result.stderr}"
        )
    return result


def write_json(path: Path, value: object, allow_nan: bool = False) -> None:
    path.write_text(json.dumps(value, allow_nan=allow_nan), encoding="utf-8")


def run_prepare(request: dict, output: Path, asset_root: Path, expected_returncode: int = 0) -> subprocess.CompletedProcess[str]:
    request_path = output.with_suffix(".request.json")
    write_json(request_path, request)
    return command([
        "python3", "-m", "soarm_named_place_request.prepare",
        "--request", str(request_path), "--output", str(output), "--asset-root", str(asset_root),
    ], expected_returncode)


def run_verify(payload: Path, asset_root: Path, expected_returncode: int = 0) -> subprocess.CompletedProcess[str]:
    return command([
        "python3", "-m", "soarm_named_place_request.prepare",
        "--verify-prepared", str(payload), "--asset-root", str(asset_root),
    ], expected_returncode)


def assert_rejected(result: subprocess.CompletedProcess[str], expected_error: str) -> None:
    assert expected_error in result.stderr, result.stderr


def verify_ros_consumer(output_paths: list[Path]) -> None:
    expected_image = json.loads(LOCK.read_text(encoding="utf-8"))["runtime_evidence"]["image_id"]
    assert expected_image == IMAGE_ID
    observed_image = command([
        "docker", "image", "inspect", IMAGE_ID, "--format", "{{.Id}}",
    ]).stdout.strip()
    assert observed_image == IMAGE_ID, observed_image
    inspect_script = """
import json
import sys
from nav2_msgs.action import NavigateToPose
from rclpy.serialization import deserialize_message, serialize_message
from soarm_named_place_request.prepare import verify_prepared
asset_root = sys.argv[1]
for filename in sys.argv[2:]:
    prepared = json.load(open(filename, encoding='utf-8'))
    goal_data = verify_prepared(prepared, __import__('pathlib').Path(asset_root))
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
    assert replayed.pose.pose.position.x == goal_data['position']['x']
    assert replayed.pose.pose.position.y == goal_data['position']['y']
    assert replayed.pose.pose.position.z == 0.0
    assert replayed.pose.pose.orientation.z == goal_data['orientation']['z']
    assert replayed.pose.pose.orientation.w == goal_data['orientation']['w']
print('verified and serialized', len(sys.argv) - 2, 'position-only NavigateToPose goals')
"""
    inspect_path = output_paths[0].parent / "inspect_navigate_to_pose.py"
    inspect_path.write_text(inspect_script, encoding="utf-8")
    command([
        "docker", "run", "--rm", "--network", "none",
        "-v", f"{REPOSITORY_ROOT}:{REPOSITORY_ROOT}:ro",
        "-v", f"{BASELINE}:{BASELINE}:ro",
        "-v", "/tmp:/tmp:ro",
        "--entrypoint", "bash", IMAGE_ID, "-lc",
        "source /opt/ros/humble/setup.bash && PYTHONPATH=\"$PYTHONPATH:" + str(MODULE_ROOT) +
        "\" python3 " + str(inspect_path) + " " + str(BASELINE) + " " + " ".join(map(str, output_paths)),
    ])


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="soarm-nav-request-") as temporary:
        temporary_root = Path(temporary)
        outputs = []
        catalog = json.loads((REPOSITORY_ROOT / "artifacts/rooms-complete-reference.json").read_text(encoding="utf-8"))
        expected = {room["id"]: room["goal_pose"] for room in catalog["rooms"]}
        for place_id in ("room-1", "room-2", "room-3", "mian"):
            output = temporary_root / f"{place_id}.json"
            run_prepare({"place_id": place_id, "frame_id": "map", "require_heading": False}, output, BASELINE)
            prepared = json.loads(output.read_text(encoding="utf-8"))
            x, y, yaw = expected[place_id]
            assert prepared["require_heading"] is False
            assert prepared["map_goal"]["position"] == {"x": float(x), "y": float(y), "z": 0.0}
            assert math.isclose(prepared["map_goal"]["orientation"]["z"], math.sin(yaw / 2.0))
            assert math.isclose(prepared["map_goal"]["orientation"]["w"], math.cos(yaw / 2.0))
            outputs.append(output)
        verify_ros_consumer(outputs)

        rejected = temporary_root / "rejected.json"
        false_heading = {"place_id": "room-1", "frame_id": "map", "require_heading": False}
        assert_rejected(run_prepare({**false_heading, "require_heading": True}, rejected, BASELINE, 2), "unsupported_heading")
        assert_rejected(run_prepare({**false_heading, "place_id": "demo_target"}, rejected, BASELINE, 2), "unsupported place_id")
        assert_rejected(run_prepare({**false_heading, "frame_id": "camera_rgb_optical_frame"}, rejected, BASELINE, 2), "no transform is available")
        assert_rejected(run_prepare({"place_id": "room-1", "require_heading": False}, rejected, BASELINE, 2), "request fields")

        changed_root = temporary_root / "changed-assets"
        changed_pgm = changed_root / "ros2_ws/src/soarm_simulation/maps/house_floorplan.pgm"
        changed_pgm.parent.mkdir(parents=True)
        changed_pgm.write_bytes((BASELINE / "ros2_ws/src/soarm_simulation/maps/house_floorplan.pgm").read_bytes() + b"changed")
        assert_rejected(run_prepare(false_heading, rejected, changed_root, 2), "locked asset changed")
        missing_root = temporary_root / "missing-pcd"
        missing_pgm = missing_root / "ros2_ws/src/soarm_simulation/maps/house_floorplan.pgm"
        missing_pgm.parent.mkdir(parents=True)
        missing_pgm.write_bytes((BASELINE / "ros2_ws/src/soarm_simulation/maps/house_floorplan.pgm").read_bytes())
        assert_rejected(run_prepare(false_heading, rejected, missing_root, 2), "locked asset missing")

        for axis in ("x", "y", "z"):
            forged = json.loads(outputs[0].read_text(encoding="utf-8"))
            forged["map_goal"]["position"][axis] = 99.0
            forged_path = temporary_root / f"forged-{axis}.json"
            write_json(forged_path, forged)
            assert_rejected(run_verify(forged_path, BASELINE, 2), "does not exactly match catalog pose")
        for field, value, reason in (
                ("status", "action_ready", "status is not prepared_for_serialization"),
                ("asset_metadata", {}, "unsupported fields")):
            forged = json.loads(outputs[0].read_text(encoding="utf-8"))
            forged[field] = value
            forged_path = temporary_root / f"forged-{field}.json"
            write_json(forged_path, forged)
            assert_rejected(run_verify(forged_path, BASELINE, 2), reason)
        forged = json.loads(outputs[0].read_text(encoding="utf-8"))
        forged["map_goal"]["orientation"]["w"] = 0.0
        forged_path = temporary_root / "forged-quat.json"
        write_json(forged_path, forged)
        assert_rejected(run_verify(forged_path, BASELINE, 2), "not a unit quaternion")
        forged = json.loads(outputs[0].read_text(encoding="utf-8"))
        forged["map_goal"]["orientation"]["z"] *= -1.0
        forged["map_goal"]["orientation"]["w"] *= -1.0
        forged_path = temporary_root / "forged-unit-quat.json"
        write_json(forged_path, forged)
        assert_rejected(run_verify(forged_path, BASELINE, 2), "does not exactly match catalog pose")
        forged = json.loads(outputs[0].read_text(encoding="utf-8"))
        forged["map_goal"]["position"]["x"] = float("nan")
        forged_path = temporary_root / "forged-nonfinite.json"
        write_json(forged_path, forged, allow_nan=True)
        assert_rejected(run_verify(forged_path, BASELINE, 2), "finite non-boolean number")
        forged = json.loads(outputs[0].read_text(encoding="utf-8"))
        forged["map_goal"]["position"]["x"] = True
        forged_path = temporary_root / "forged-bool.json"
        write_json(forged_path, forged)
        assert_rejected(run_verify(forged_path, BASELINE, 2), "finite non-boolean number")
        forged = json.loads(outputs[0].read_text(encoding="utf-8"))
        forged["asset_manifest_sha256"] = "0" * 64
        forged_path = temporary_root / "forged-alt-lock.json"
        write_json(forged_path, forged)
        assert_rejected(run_verify(forged_path, BASELINE, 2), "asset manifest hash")
        result = command([
            "python3", "-m", "soarm_named_place_request.prepare",
            "--request", str(outputs[0].with_suffix(".request.json")),
            "--output", str(rejected), "--asset-root", str(BASELINE),
            "--lock", "/tmp/alternate-lock.json",
        ], expected_returncode=2)
        assert "unrecognized arguments: --lock" in result.stderr, result.stderr


if __name__ == "__main__":
    main()
    print("named-place request consumer compatibility tests passed")
