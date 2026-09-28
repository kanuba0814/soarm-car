#!/usr/bin/env python3
"""Render a reproducible B06 eight-system comparison from verified local videos.

The renderer deliberately never substitutes one system's output for another:
missing visual artifacts become labelled status cards and are recorded in the
manifest.  The checked HOV-SG termination card is selected ahead of any later
verified HOV-SG visual artifact, so its status cannot be mistaken for a run.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import tempfile
from pathlib import Path
from typing import Any

TILE_WIDTH, TILE_HEIGHT, FPS = 640, 480, 10
SYSTEMS = (
    {
        "name": "FARM", "result_type": "inventory", "recall": "strict recall lower bound: 8%",
        "candidates": ("artifacts/perception-dev/benchmark-runs/inventory-v1/FARM-B06-cpu-stride5-overlay.mp4",),
        "missing": "No verified FARM B06 video found.",
    },
    {
        "name": "ConceptGraphs", "result_type": "audit", "recall": "strict recall lower bound: 4%",
        "candidates": ("artifacts/perception-dev/results/ConceptGraphs-B06-stride5/audit/map_audit_overlay.mp4",),
        "missing": "No verified ConceptGraphs B06 audit overlay found.",
    },
    {
        "name": "Open3DIS", "result_type": "2D proposals", "recall": "strict recall lower bound: 4%",
        "candidates": ("artifacts/perception-dev/results/Open3DIS-B06-stride8/B06-stride8-2d-proposals-overlay.mp4",),
        "missing": "No verified Open3DIS B06 2D-proposals overlay found. Proposal count is not object success.",
    },
    {
        "name": "OVIR-3D", "result_type": "inventory", "recall": "strict recall lower bound: 0%",
        "candidates": ("artifacts/perception-dev/benchmark-runs/inventory-v1/OVIR-3D-B06-cpu-overlay.mp4",),
        "missing": "No verified OVIR-3D B06 overlay found.",
    },
    {
        "name": "OpenFusion", "result_type": "TSDF geometry", "recall": "strict recall lower bound: N/A",
        "candidates": (
            "artifacts/perception-dev/results/OpenFusion-B06-tsdf-geometry-rotate.mp4",
            "artifacts/perception-dev/results/OpenFusion-B06-geometry-rotate.mp4",
            "artifacts/perception-dev/results/OpenFusion-B06-rotate.mp4",
        ),
        "missing": "PENDING: no verified OpenFusion TSDF geometry rotation video. TSDF geometry is not inventory.",
    },
    {
        "name": "FindAnything", "result_type": "N/A", "recall": "strict recall lower bound: N/A",
        "candidates": (),
        "missing": "N/A: B06 lacks the calibrated synchronized stereo cam1 stream and extended EuRoC cam0/cam1 layout.",
    },
    {
        "name": "HOV-SG", "result_type": "TERMINATED status", "recall": "strict recall lower bound: N/A",
        "candidates": (
            "artifacts/perception-dev/results/comparison/HOV-SG-B06-termination-status.mp4",
            "artifacts/perception-dev/results/HOV-SG-B06-overlay.mp4",
            "artifacts/perception-dev/results/HOV-SG-B06-geometry-rotate.mp4",
            "artifacts/perception-dev/results/HOV-SG-B06-rotate.mp4",
        ),
        "missing": "TERMINATED: full 213-frame run stopped after over 3 hours during sequential 3D mask fusion at about 12.4 GB RAM (87%). Three-frame smoke retained 43 mask point clouds, but no projection is shown because verified mask geometry is unavailable locally. ScanNet path skips hierarchy; inventory N/A.",
    },
    {
        "name": "HomeRobot", "result_type": "BLOCKED", "recall": "strict recall lower bound: N/A",
        "candidates": (),
        "missing": "BLOCKED: pinned spot-sim2real commit cannot be fetched (upstream: not our ref). Not a B06 RGB-D consumer.",
    },
)


def run(command: list[str]) -> None:
    subprocess.run(command, check=True)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def probe(path: Path) -> dict[str, Any] | None:
    result = subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "format=duration", "-show_entries",
         "stream=codec_type,width,height,r_frame_rate", "-of", "json", str(path)],
        text=True, capture_output=True,
    )
    if result.returncode:
        return None
    data = json.loads(result.stdout)
    if not any(stream.get("codec_type") == "video" for stream in data.get("streams", [])):
        return None
    return data


def escape_drawtext(value: str) -> str:
    return value.replace("\\", "\\\\").replace(":", "\\:").replace("'", "\\'").replace("%", "\\\\%")


def duration_seconds(probe_data: dict[str, Any]) -> float:
    return float(probe_data["format"]["duration"])


def render_video_tile(source: Path, output: Path, system: dict[str, Any], duration: float, looped: bool) -> None:
    labels = [system["name"], system["result_type"], system["recall"]]
    if looped:
        labels.append("LOOPED")
    filter_graph = (
        f"scale={TILE_WIDTH}:{TILE_HEIGHT}:force_original_aspect_ratio=decrease,"
        f"pad={TILE_WIDTH}:{TILE_HEIGHT}:(ow-iw)/2:(oh-ih)/2:color=black,"
        "drawbox=x=0:y=0:w=iw:h=82:color=black@0.78:t=fill,"
        f"drawtext=text='{escape_drawtext(labels[0])}':x=16:y=12:fontsize=28:fontcolor=white,"
        f"drawtext=text='{escape_drawtext(labels[1])}':x=16:y=46:fontsize=20:fontcolor=0x9fd3ff,"
        f"drawtext=text='{escape_drawtext(labels[2])}':x=16:y=h-33:fontsize=18:fontcolor=white:box=1:boxcolor=black@0.7:boxborderw=6"
    )
    if looped:
        filter_graph += ",drawtext=text='LOOPED':x=w-106:y=h-34:fontsize=18:fontcolor=yellow:box=1:boxcolor=black@0.7:boxborderw=6"
    run(["ffmpeg", "-y", "-stream_loop", "-1", "-i", str(source), "-t", f"{duration:.3f}", "-vf", filter_graph,
         "-r", str(FPS), "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(output)])


def render_status_tile(output: Path, system: dict[str, Any], duration: float) -> None:
    status = "PENDING" if "PENDING" in system["result_type"] or system["name"] == "OpenFusion" else system["result_type"]
    reason = system["missing"]
    wrapped = []
    words = reason.split()
    line = ""
    for word in words:
        if len(line) + len(word) + 1 > 51:
            wrapped.append(line)
            line = word
        else:
            line = f"{line} {word}".strip()
    if line:
        wrapped.append(line)
    graph = [
        f"drawbox=x=20:y=20:w={TILE_WIDTH - 40}:h={TILE_HEIGHT - 40}:color=0x4b5a6b:t=2",
        f"drawtext=text='{escape_drawtext(system['name'])}':x=44:y=56:fontsize=30:fontcolor=white",
        f"drawtext=text='{escape_drawtext(system['result_type'])} | {escape_drawtext(status)}':x=44:y=99:fontsize=21:fontcolor=0xffcc75",
    ]
    for index, text in enumerate(wrapped[:7]):
        graph.append(f"drawtext=text='{escape_drawtext(text)}':x=44:y={155 + index * 31}:fontsize=19:fontcolor=0xd7dde5")
    graph.append(f"drawtext=text='{escape_drawtext(system['recall'])}':x=44:y=432:fontsize=18:fontcolor=0xbfcbd9")
    run(["ffmpeg", "-y", "-f", "lavfi", "-i", f"color=c=0x19202a:s={TILE_WIDTH}x{TILE_HEIGHT}:r={FPS}", "-t", f"{duration:.3f}",
         "-vf", ",".join(graph), "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(output)])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/perception-dev/results/comparison"))
    args = parser.parse_args()
    root = args.root.resolve()
    output_dir = (root / args.output_dir).resolve() if not args.output_dir.is_absolute() else args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    selections: list[dict[str, Any]] = []
    for system in SYSTEMS:
        selected, probe_data = None, None
        for candidate in system["candidates"]:
            path = root / candidate
            if path.is_file() and (candidate_probe := probe(path)) is not None:
                selected, probe_data = path, candidate_probe
                break
        selections.append({"system": system, "source": selected, "probe": probe_data})

    longest = max((duration_seconds(item["probe"]) for item in selections if item["probe"]), default=21.3)
    # Status cards need the same number of frames; ceil prevents truncating a real source.
    output_duration = max(longest, 0.1)
    video = output_dir / "B06-eight-system-comparison-preview.mp4"
    contact = output_dir / "B06-eight-system-comparison-contact-sheet.jpg"
    manifest_path = output_dir / "B06-eight-system-comparison-manifest.json"
    sums_path = output_dir / "SHA256SUMS"

    with tempfile.TemporaryDirectory(prefix="b06-comparison-", dir=output_dir) as temporary:
        temp = Path(temporary)
        tiles = []
        for index, item in enumerate(selections):
            system, source, probe_data = item["system"], item["source"], item["probe"]
            tile = temp / f"tile-{index}.mp4"
            if source is not None:
                render_video_tile(source, tile, system, output_duration, duration_seconds(probe_data) + 0.001 < output_duration)
            else:
                render_status_tile(tile, system, output_duration)
            tiles.append(tile)
        inputs = [part for tile in tiles for part in ("-i", str(tile))]
        layout = "|".join(f"{(index % 2) * TILE_WIDTH}_{(index // 2) * TILE_HEIGHT}" for index in range(8))
        run(["ffmpeg", "-y", *inputs, "-filter_complex", f"xstack=inputs=8:layout={layout}[v]", "-map", "[v]",
             "-t", f"{output_duration:.3f}", "-r", str(FPS), "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(video)])

    # Eight evenly spaced final-video frames form a directly inspectable contact sheet.
    run(["ffmpeg", "-y", "-i", str(video), "-vf", f"fps=8/{output_duration:.6f},scale=320:480,tile=4x2", "-frames:v", "1", str(contact)])
    final_probe = probe(video)
    if final_probe is None:
        raise RuntimeError("ffprobe could not read rendered comparison video")

    sources = []
    for item in selections:
        source = item["source"]
        sources.append({
            "system": item["system"]["name"], "result_type": item["system"]["result_type"], "recall": item["system"]["recall"],
            "source": str(source.relative_to(root)) if source else None,
            "source_sha256": sha256(source) if source else None,
            "source_duration_seconds": duration_seconds(item["probe"]) if item["probe"] else None,
            "looped": bool(source and duration_seconds(item["probe"]) + 0.001 < output_duration),
            "status_card_reason": None if source else item["system"]["missing"],
        })
    manifest = {
        "schema": "soarm.b06_eight_system_comparison.v1",
        "sequence": "B06",
        "ground_truth": "25-item agent draft; pending independent review; not human/hidden ground truth",
        "tile": {"width": TILE_WIDTH, "height": TILE_HEIGHT, "layout": "2 columns x 4 rows", "fps": FPS},
        "video": {"path": video.name, "sha256": sha256(video), "duration_seconds": duration_seconds(final_probe), "ffprobe": final_probe},
        "contact_sheet": {"path": contact.name, "sha256": sha256(contact)},
        "sources": sources,
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    sums_path.write_text("".join(f"{sha256(path)}  {path.name}\n" for path in (video, contact, manifest_path)), encoding="utf-8")
    print(json.dumps({"video": str(video), "contact_sheet": str(contact), "manifest": str(manifest_path), "duration": manifest["video"]["duration_seconds"]}, indent=2))


if __name__ == "__main__":
    main()
