#!/usr/bin/env python3
"""Export and offline-verify an immutable Nav2 dirty-worktree recovery snapshot.

This tool only reads the source worktree. It deliberately archives a small,
explicit set of implementation and consumer inputs instead of its artifacts
directory wholesale. ``--verify`` restores into a caller-provided empty path;
it never removes worktrees.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
from typing import Any

EXPECTED_HEAD = "41463f9a07dc317eb837558ad55b137fb92e35eb"
ARCHIVE_BUDGET = 100 * 1024 * 1024
INCLUDED = (
    "ros2_ws/src/soarm_navigation/behavior_trees/navigate_position_only_holonomic.xml",
    "ros2_ws/src/soarm_navigation/config/house_nav2_mppi_shim.yaml",
    "scripts/generate_house_reference_pcd.py",
    "scripts/nav2-clean-planar-probe.py",
    "scripts/nav2-clean-route-consumer.py",
    "scripts/nav2-clean-yaw-probe.py",
    "scripts/run-nav2-clean-comparison.sh",
    "scripts/run-nav2-clean-planar-probe.sh",
    "scripts/run-nav2-clean-yaw-probe.sh",
    "artifacts/house_widened_reference.pcd",
    "artifacts/house_current_reference.pcd",
    "artifacts/rooms-complete-reference.json",
    "artifacts/nav2-clean-direct-mppi-20260905T173147Z/summary.json",
    "artifacts/nav2-clean-direct-mppi-20260905T173147Z/snapshot.json",
    "artifacts/nav2-clean-direct-mppi-20260905T173147Z/direct_mppi/metrics.json",
)
CONSUMER_SOURCES = (
    "ros2_ws/src/soarm_navigation/soarm_navigation/launch/house_fastlio_nav2.launch.py",
    "ros2_ws/src/soarm_navigation/config/house_nav2_mppi.yaml",
    "ros2_ws/src/soarm_simulation/models/house_floorplan/model.sdf",
    "ros2_ws/src/soarm_simulation/models/kuruma_ros2_control/model.sdf",
    "ros2_ws/src/soarm_simulation/worlds/house_floorplan_ros2_control.sdf",
    "ros2_ws/src/soarm_simulation/maps/house_floorplan.yaml",
    "ros2_ws/src/soarm_simulation/maps/house_floorplan.pgm",
    "ros2_ws/src/soarm_simulation/config/kuruma_ros2_control_controllers.yaml",
    "vendor/localization/src/FAST_LIO_LOCALIZATION_ROS2/config/soarm_house_localization.yaml",
    "ros2_ws/src/soarm_navigation/soarm_navigation/localization_cmd_gate.py",
    "scripts/nav2-clean-route-consumer.py",
    "artifacts/house_widened_reference.pcd",
    "artifacts/rooms-complete-reference.json",
)
RECEIPT_PREFIX = "artifacts/nav2-clean-direct-mppi-20260905T173147Z/"


def run(*command: str, cwd: Path) -> str:
    return subprocess.check_output(command, cwd=cwd, text=True, stderr=subprocess.STDOUT)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def file_record(root: Path, relative: str, reason: str) -> dict[str, Any]:
    path = root / relative
    if not path.is_file():
        raise RuntimeError(f"required allowlist input missing: {relative}")
    return {"path": relative, "bytes": path.stat().st_size, "sha256": digest(path), "reason": reason}


def image_identity(source: Path) -> dict[str, Any]:
    image = "soarm-car-ros2-dev"
    try:
        value = run("docker", "image", "inspect", "--format={{.Id}}", image, cwd=source).strip()
        return {"name": image, "id": value, "inspection": "docker image inspect (read-only)"}
    except (FileNotFoundError, subprocess.CalledProcessError) as error:
        return {"name": image, "id": None, "inspection": f"unavailable: {type(error).__name__}"}


def source_status(source: Path) -> dict[str, str]:
    return {
        "porcelain_v1": run("git", "status", "--porcelain=v1", cwd=source),
        "staged_porcelain_v1": run("git", "diff", "--cached", "--name-status", cwd=source),
    }


def excluded_artifacts(source: Path) -> list[dict[str, Any]]:
    excluded = []
    for path in sorted((source / "artifacts").rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(source).as_posix()
        if relative in INCLUDED:
            continue
        reason = "unrelated raw log/container output" if "/container" in relative or "/cleanup/" in relative else "not in explicit recovery allowlist"
        entry: dict[str, Any] = {"path": relative, "bytes": path.stat().st_size, "reason": reason}
        if path.stat().st_size >= 1024 * 1024:
            entry["sha256"] = digest(path)
            entry["required_dependency"] = relative.endswith("house_current_reference.pcd")
        excluded.append(entry)
    return excluded


def export(source: Path, output: Path) -> None:
    if output.exists():
        raise RuntimeError(f"refusing to overwrite existing snapshot: {output}")
    head = run("git", "rev-parse", "HEAD", cwd=source).strip()
    if head != EXPECTED_HEAD:
        raise RuntimeError(f"source HEAD must be {EXPECTED_HEAD}, got {head}")
    before = source_status(source)
    patch = subprocess.check_output(["git", "diff", "--binary", "HEAD"], cwd=source)
    output.mkdir(parents=True)
    try:
        (output / "tracked-head.patch").write_bytes(patch)
        included = [file_record(source, relative, "explicit consumed implementation/input allowlist") for relative in INCLUDED]
        archive_size = sum(entry["bytes"] for entry in included)
        if archive_size > ARCHIVE_BUDGET:
            raise RuntimeError(f"allowlist is {archive_size} bytes, over {ARCHIVE_BUDGET} byte budget")
        archive_path = output / "untracked-allowlist.tar.gz"
        with tarfile.open(archive_path, "w:gz", format=tarfile.PAX_FORMAT) as archive:
            for entry in included:
                archive.add(source / entry["path"], arcname=entry["path"], recursive=False)
        manifest = {
            "format": 1,
            "source": str(source),
            "head": head,
            "base": head,
            "source_status_before": before,
            "tracked_patch": {"path": "tracked-head.patch", "bytes": len(patch), "sha256": digest(output / "tracked-head.patch")},
            "archive": {"path": archive_path.name, "bytes": archive_path.stat().st_size, "sha256": digest(archive_path), "budget_bytes": ARCHIVE_BUDGET},
            "included_files": included,
            "excluded_files": excluded_artifacts(source),
            "consumer_sources": [file_record(source, relative, "must match restored consumer seam") for relative in CONSUMER_SOURCES],
            "image": image_identity(source),
            "receipt": {"path": RECEIPT_PREFIX + "summary.json", "expected_completed_goals": 4, "status": "recorded only; no rerun"},
        }
        after = source_status(source)
        manifest["source_status_after"] = after
        manifest["source_unchanged"] = before == after
        if not manifest["source_unchanged"]:
            raise RuntimeError("source status changed during read-only export")
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    except BaseException:
        shutil.rmtree(output)
        raise


def safe_extract(archive_path: Path, destination: Path) -> None:
    with tarfile.open(archive_path, "r:gz") as archive:
        members = archive.getmembers()
        for member in members:
            path = Path(member.name)
            if path.is_absolute() or ".." in path.parts or member.issym() or member.islnk() or not member.isfile():
                raise RuntimeError(f"unsafe archive member: {member.name}")
        archive.extractall(destination, members=members, filter="fully_trusted")


def verify(snapshot: Path, restore: Path) -> None:
    if restore.exists():
        raise RuntimeError(f"restore path must not exist: {restore}")
    manifest = json.loads((snapshot / "manifest.json").read_text())
    subprocess.run(["git", "worktree", "add", "--detach", str(restore), manifest["base"]], cwd=snapshot.parent.parent, check=True)
    subprocess.run(["git", "apply", "--binary", str(snapshot / manifest["tracked_patch"]["path"])], cwd=restore, check=True)
    safe_extract(snapshot / manifest["archive"]["path"], restore)
    for record in manifest["included_files"] + manifest["consumer_sources"]:
        path = restore / record["path"]
        if not path.is_file() or digest(path) != record["sha256"]:
            raise RuntimeError(f"restored hash mismatch: {record['path']}")
    for relative in ("ros2_ws/src/soarm_navigation/config/house_nav2_mppi.yaml", "ros2_ws/src/soarm_navigation/config/house_nav2_mppi_shim.yaml", "vendor/localization/src/FAST_LIO_LOCALIZATION_ROS2/config/soarm_house_localization.yaml"):
        subprocess.run([sys.executable, "-c", "import sys,yaml; yaml.safe_load(open(sys.argv[1]))", str(restore / relative)], check=True)
    subprocess.run([sys.executable, "-m", "py_compile", str(restore / "scripts/nav2-clean-route-consumer.py"), str(restore / "ros2_ws/src/soarm_navigation/soarm_navigation/launch/house_fastlio_nav2.launch.py")], check=True)
    rooms = json.loads((restore / "artifacts/rooms-complete-reference.json").read_text())
    if not {"room-1", "room-2", "room-3", "mian"}.issubset({room["id"] for room in rooms["rooms"]}):
        raise RuntimeError("room catalog does not contain saved route")
    receipt = json.loads((restore / RECEIPT_PREFIX / "summary.json").read_text())
    route = receipt.get("cases", {}).get("direct_mppi", {}).get("route", [])
    if len(route) != 4 or any(goal.get("status") != 4 for goal in route):
        raise RuntimeError("saved receipt does not contain four SUCCEEDED goals")
    if not (restore / "artifacts/house_widened_reference.pcd").read_bytes().startswith(b"# .PCD v0.7"):
        raise RuntimeError("reference PCD header invalid")
    print(f"offline restore verified: {restore}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, help="read-only dirty source worktree")
    parser.add_argument("--output", type=Path, help="new snapshot directory")
    parser.add_argument("--verify", action="store_true", help="offline restore and validate an existing --output")
    parser.add_argument("--restore", type=Path, help="new disposable restore worktree path for --verify")
    args = parser.parse_args()
    if args.verify:
        if not args.output or not args.restore:
            parser.error("--verify requires --output and --restore")
        verify(args.output.resolve(), args.restore.resolve())
    else:
        if not args.source or not args.output:
            parser.error("export requires --source and --output")
        export(args.source.resolve(), args.output.resolve())


if __name__ == "__main__":
    main()
