#!/usr/bin/env python3
"""Write the auditable B06 visualization package manifest without inventing results."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path
from typing import Any


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def probe(path: Path) -> dict[str, Any]:
    completed = subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "format=duration,size", "-show_entries",
         "stream=codec_name,width,height,r_frame_rate,nb_frames", "-of", "json", str(path)],
        check=True, capture_output=True, text=True,
    )
    return json.loads(completed.stdout)


def artifact(root: Path, relative: str, kind: str, frames: str, completion: str,
             inventory: str, lower_bound: str | None = None) -> dict[str, Any]:
    path = root / relative
    record: dict[str, Any] = {
        "source_artifact": relative,
        "visualization": kind,
        "frames_sampled": frames,
        "completion_status": completion,
        "inventory_boundary": inventory,
        "strict_audited_lower_bound": lower_bound,
    }
    if path.is_file():
        record["sha256"] = sha256(path)
        if path.suffix.lower() == ".mp4":
            record["ffprobe"] = probe(path)
    else:
        record["available"] = False
    return record


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/perception-dev/results/comparison"))
    args = parser.parse_args()
    root = args.root.resolve()
    output_dir = (root / args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    hov_status = {
        "schema": "soarm.hovsg_b06_termination_status.v1",
        "system": "HOV-SG",
        "sequence": "B06_20260901-181608",
        "process": {"pid": 34508, "observed_stopped": True},
        "full_run": {
            "status": "terminated_authorized",
            "reason": "Authorized termination after more than 3 hours during sequential 3D mask fusion; no full-run output was produced.",
            "resources_at_termination": {"ram_gb_approx": 12.4, "ram_percent_approx": 87},
        },
        "smoke_run": {
            "status": "succeeded_per_authorized_run_record",
            "sampled_source_frames": [0, 100, 200],
            "retained_mask_point_clouds": 43,
            "local_artifacts_available": False,
            "projection": "not rendered: local verified mask geometry, intrinsics/pose provenance, and projection inputs are unavailable; no masks were interpolated or fabricated.",
        },
        "inventory": "N/A: ScanNet path skips hierarchy; mask proposals are not physical objects or persistent instances.",
        "historical_local_logs": [
            "artifacts/perception-dev/results/remote/B06-HOV-SG-env-create.log",
            "artifacts/perception-dev/results/remote/B06-HOV-SG-official-loader-smoke.log",
        ],
        "authoritative_smoke_full_logs": "unavailable locally because /mnt/soarm-perception is not mounted during packaging; the historical local logs predate this authorized smoke/full run and are retained without relabeling.",
    }
    hov_status_path = output_dir / "HOV-SG-B06-termination-status.json"
    hov_status_path.write_text(json.dumps(hov_status, indent=2) + "\n", encoding="utf-8")

    systems = {
        "FARM": artifact(root, "artifacts/perception-dev/benchmark-runs/inventory-v1/FARM-B06-cpu-stride5-overlay.mp4", "overlay", "stride 5; 43 of 213", "completed", "persistent inventory prediction; RGB-depth registration remains unverified", "8%"),
        "ConceptGraphs": artifact(root, "artifacts/perception-dev/results/ConceptGraphs-B06-stride5/audit/map_audit_overlay.mp4", "audit overlay", "43 source frames; rendered 43 frames", "completed", "persistent inventory prediction/audit", "4%"),
        "Open3DIS": artifact(root, "artifacts/perception-dev/results/Open3DIS-B06-stride8/B06-stride8-2d-proposals-overlay.mp4", "2D proposal overlay", "stride 8; 27 of 213", "completed", "2D proposals only in this video; proposal count is not object success", "4%"),
        "OVIR-3D": artifact(root, "artifacts/perception-dev/benchmark-runs/inventory-v1/OVIR-3D-B06-cpu-overlay.mp4", "overlay", "213 of 213", "completed", "persistent inventory prediction; labels require human adjudication", "0%"),
        "OpenFusion": artifact(root, "artifacts/perception-dev/results/OpenFusion-B06-tsdf-geometry-rotate.mp4", "TSDF geometry rotation", "geometry render; 72 video frames", "completed", "geometry only; not inventory", None),
        "FindAnything": artifact(root, "artifacts/perception-dev/benchmark-runs/inventory-v1/FindAnything-B06-status.mp4", "status", "no B06 inference", "not-applicable", "N/A: calibrated synchronized stereo input absent; no synthetic stereo", None),
        "HOV-SG": artifact(root, "artifacts/perception-dev/results/comparison/HOV-SG-B06-termination-status.mp4", "termination status", "smoke frames 0,100,200; no projected masks", "terminated", "N/A: ScanNet path skips hierarchy; no persistent inventory", None),
        "HomeRobot": artifact(root, "artifacts/perception-dev/benchmark-runs/inventory-v1/HomeRobot-B06-status.mp4", "status", "no B06 inference", "blocked/not-applicable", "N/A: not a B06 RGB-D consumer; pinned source unavailable", None),
    }
    pipelines = {
        "YOLO-World": artifact(root, "artifacts/perception-dev/results/remote/B06-yoloworld.mp4", "detector overlay", "30 sampled source frames", "completed", "not a persistent inventory system", None),
        "GroundingDINO": artifact(root, "artifacts/perception-dev/results/remote/B06-groundingdino.mp4", "detector overlay", "30 sampled source frames", "completed", "not a persistent inventory system", None),
        "YOLO-World+SAM2": artifact(root, "artifacts/perception-dev/results/remote/B06-yoloworld-sam2.mp4", "detector+mask overlay", "30 sampled source frames", "completed", "not a persistent inventory system", None),
        "GroundingDINO+SAM2": artifact(root, "artifacts/perception-dev/results/remote/B06-groundingdino-sam2.mp4", "detector+mask overlay", "30 sampled source frames", "completed", "not a persistent inventory system", None),
    }
    detector_comparison = artifact(root, "artifacts/perception-dev/results/remote/B06-four-pipelines.mp4", "four-pipeline detector comparison overlay", "30 sampled source frames", "completed; copied/preserved without re-encoding", "not a persistent inventory system", None)
    expected_detector_hash = "d3f240f37d48aed6027f430deecd3d3af3d96dc169899de39c18ce922b7a4269"
    detector_comparison["expected_sha256_from_B06_SHA256SUMS"] = expected_detector_hash
    detector_comparison["hash_matches_expected"] = detector_comparison.get("sha256") == expected_detector_hash

    comparison = artifact(root, "artifacts/perception-dev/results/comparison/B06-eight-system-comparison-preview.mp4", "eight-system comparison", "B06 timeline/status cards; 215 frames at 10 fps", "completed", "see per-system boundaries", None)
    report = {
        "schema": "soarm.b06_visualization_package.v1",
        "sequence": "B06_20260901-181608",
        "ground_truth_boundary": "25-item agent draft pending independent review; not human/hidden ground truth.",
        "eight_system_comparison": comparison,
        "benchmark_systems": systems,
        "detector_pipelines": pipelines,
        "four_pipeline_comparison": detector_comparison,
        "hov_status": {"path": hov_status_path.name, "sha256": sha256(hov_status_path)},
    }
    report_path = output_dir / "B06-visualization-package-manifest.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    paths = sorted(output_dir.glob("*.mp4")) + sorted(output_dir.glob("*.jpg")) + sorted(output_dir.glob("*.json"))
    (output_dir / "SHA256SUMS").write_text("".join(f"{sha256(path)}  {path.name}\n" for path in paths), encoding="utf-8")
    print(json.dumps({"report": str(report_path), "hov_status": str(hov_status_path)}, indent=2))


if __name__ == "__main__":
    main()
