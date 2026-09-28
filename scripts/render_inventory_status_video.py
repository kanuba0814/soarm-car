#!/usr/bin/env python3
"""Render an explicit benchmark status panel instead of fabricating an overlay."""

from __future__ import annotations

import argparse
import json
import subprocess
import textwrap
from pathlib import Path


def nested(record: dict, *keys: str, default: str = "") -> str:
    value = record
    for key in keys:
        if not isinstance(value, dict):
            return default
        value = value.get(key)
    return default if value is None else str(value)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("record", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--duration", type=float, default=21.3)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    record = json.loads(args.record.read_text(encoding="utf-8"))
    system = str(record.get("system", "Unknown system"))
    result = record.get("b06_result") or {}
    if record.get("full_run"):
        full_run = record["full_run"]
        smoke_run = record.get("smoke_run", {})
        result = {
            "status": full_run.get("status", "terminated"),
            "reason": (
                f"{full_run.get('reason', 'No full-run output.')} "
                f"Smoke frames {smoke_run.get('sampled_source_frames', [])} retained "
                f"{smoke_run.get('retained_mask_point_clouds', 'no recorded')} mask point clouds. "
                "No projection: verified local mask geometry is unavailable."
            ),
            "inventory_score": None,
        }
    status = str(result.get("status") or nested(record, "official_consumer", "status", default="blocked"))
    reason = str(result.get("reason") or nested(record, "official_consumer", "blocker", default="No result"))
    revision = str(record.get("source_revision", record.get("schema", "unknown")))[:12]
    score = result.get("inventory_score")

    def escaped(value: str) -> str:
        return value.replace("\\", "\\\\").replace(":", "\\:").replace("'", "\\'").replace("%", "\\%")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    status_color = "0x50beff" if status == "not-applicable" else "0x5a96ff"
    filters = [
        f"drawbox=x=24:y=24:w={args.width - 48}:h={args.height - 48}:color=0x373e49:t=2",
        f"drawtext=text='{escaped(system)}':x=48:y=56:fontsize=34:fontcolor=white",
        f"drawtext=text='{escaped(status.upper())}':x=48:y=108:fontsize=24:fontcolor={status_color}",
    ]
    for index, paragraph in enumerate(textwrap.wrap(reason, width=67)[:7]):
        filters.append(f"drawtext=text='{escaped(paragraph)}':x=48:y={164 + index * 27}:fontsize=18:fontcolor=0xd7dce4")
    score_text = "inventory score: N/A" if score is None else f"inventory score: {score}"
    filters.extend([
        f"drawtext=text='{escaped(score_text)}':x=48:y={args.height - 78}:fontsize=20:fontcolor=0xb4beca",
        f"drawtext=text='source\\: {escaped(revision)} | B06':x=48:y={args.height - 48}:fontsize=17:fontcolor=0x87919e",
    ])
    subprocess.run([
        "ffmpeg", "-y", "-f", "lavfi", "-i", f"color=c=0x191c22:s={args.width}x{args.height}:r={args.fps}",
        "-t", str(args.duration), "-vf", ",".join(filters), "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(args.output),
    ], check=True)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
