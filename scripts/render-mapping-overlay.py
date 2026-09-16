#!/usr/bin/env python3
"""Render synchronized DC1 RGB/depth and SLAM trajectory overlays."""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


WIDTH = 1280
HEIGHT = 720
CAMERA_BOX = (20, 80, 780, 650)
MAP_BOX = (800, 80, 1260, 540)
STATUS_BOX = (800, 558, 1260, 650)
BG = (9, 14, 22)
PANEL = (16, 24, 36)
GRID = (49, 66, 84)
TEXT = (232, 238, 245)
MUTED = (142, 159, 177)
ORB = (35, 211, 238)
RTAB = (255, 167, 52)
BAD = (244, 79, 91)
GOOD = (79, 217, 137)


def load_font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    path = Path("/usr/share/fonts/truetype/dejavu") / name
    return ImageFont.truetype(str(path), size)


F14 = load_font(14)
F16 = load_font(16)
F18 = load_font(18)
F20B = load_font(20, True)
F24B = load_font(24, True)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def depth_color(depth_mm: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    depth_m = depth_mm.astype(np.float32) / 1000.0
    valid = (depth_m >= 0.3) & (depth_m <= 5.0)
    t = np.clip((depth_m - 0.3) / 4.7, 0.0, 1.0)
    # Turbo-like blue -> cyan -> yellow -> red ramp, implemented without extra deps.
    stops = np.array(
        [[42, 32, 146], [30, 132, 220], [42, 209, 171], [242, 221, 63], [214, 62, 48]],
        dtype=np.float32,
    )
    pos = t * (len(stops) - 1)
    lo = np.floor(pos).astype(np.int16)
    hi = np.minimum(lo + 1, len(stops) - 1)
    frac = (pos - lo)[..., None]
    colors = stops[lo] * (1.0 - frac) + stops[hi] * frac
    return colors.astype(np.uint8), valid


def camera_frame(rgb_path: Path, depth_path: Path) -> Image.Image:
    rgb = np.asarray(Image.open(rgb_path).convert("RGB"), dtype=np.uint8).copy()
    depth = np.asarray(Image.open(depth_path), dtype=np.uint16)
    colors, valid = depth_color(depth)
    top = rgb[: depth.shape[0]]
    alpha = 0.40
    top[valid] = (top[valid] * (1.0 - alpha) + colors[valid] * alpha).astype(np.uint8)
    frame = Image.fromarray(rgb)
    return frame.resize((CAMERA_BOX[2] - CAMERA_BOX[0], CAMERA_BOX[3] - CAMERA_BOX[1]), Image.Resampling.LANCZOS)


def split_segments(rows: list[dict[str, str]], valid, group=None, jump_limit=None):
    segments: list[list[tuple[int, float, float]]] = []
    current: list[tuple[int, float, float]] = []
    previous_group = None
    previous_point = None
    for index, row in enumerate(rows):
        point = (float(row["x"]), float(row["z"]))
        row_group = group(row) if group else None
        jump = (
            previous_point is not None
            and jump_limit is not None
            and math.dist(point, previous_point) > jump_limit
        )
        if not valid(row) or (group and previous_group is not None and row_group != previous_group) or jump:
            if len(current) >= 2:
                segments.append(current)
            current = []
        if valid(row):
            current.append((index, point[0], point[1]))
            previous_group = row_group
            previous_point = point
        else:
            previous_point = None
    if len(current) >= 2:
        segments.append(current)
    return segments


def world_bounds(*segments_groups):
    points = [(x, z) for groups in segments_groups for segment in groups for _, x, z in segment]
    xs = [p[0] for p in points]
    zs = [p[1] for p in points]
    xmin, xmax = min(xs), max(xs)
    zmin, zmax = min(zs), max(zs)
    span = max(xmax - xmin, zmax - zmin, 1.0) * 1.12
    cx = (xmin + xmax) / 2.0
    cz = (zmin + zmax) / 2.0
    return cx - span / 2, cx + span / 2, cz - span / 2, cz + span / 2


def map_point(x: float, z: float, bounds) -> tuple[int, int]:
    x0, y0, x1, y1 = MAP_BOX
    pad = 34
    xmin, xmax, zmin, zmax = bounds
    px = x0 + pad + (x - xmin) / (xmax - xmin) * (x1 - x0 - 2 * pad)
    py = y1 - pad - (z - zmin) / (zmax - zmin) * (y1 - y0 - 2 * pad)
    return round(px), round(py)


def draw_grid(draw: ImageDraw.ImageDraw, bounds) -> None:
    x0, y0, x1, y1 = MAP_BOX
    draw.rounded_rectangle(MAP_BOX, radius=14, fill=PANEL, outline=(38, 53, 70), width=2)
    xmin, xmax, zmin, zmax = bounds
    step = 0.5
    for x in np.arange(math.ceil(xmin / step) * step, xmax + 0.001, step):
        px, _ = map_point(float(x), zmin, bounds)
        draw.line((px, y0 + 28, px, y1 - 28), fill=GRID, width=1)
    for z in np.arange(math.ceil(zmin / step) * step, zmax + 0.001, step):
        _, py = map_point(xmin, float(z), bounds)
        draw.line((x0 + 28, py, x1 - 28, py), fill=GRID, width=1)
    draw.text((x0 + 16, y0 + 12), "TOP-DOWN TRAJECTORY  (0.5 m grid)", font=F14, fill=MUTED)
    if xmin <= 0 <= xmax and zmin <= 0 <= zmax:
        px, py = map_point(0, 0, bounds)
        draw.ellipse((px - 5, py - 5, px + 5, py + 5), fill=TEXT)
        draw.text((px + 8, py - 10), "origin", font=F14, fill=MUTED)


def draw_segments(draw, segments, upto, bounds, color, width):
    latest = None
    for segment in segments:
        visible = [(i, x, z) for i, x, z in segment if i <= upto]
        if len(visible) >= 2:
            points = [map_point(x, z, bounds) for _, x, z in visible]
            draw.line(points, fill=color, width=width, joint="curve")
            latest = points[-1]
    return latest


def status_pill(draw, xy, label, ok, color):
    x, y = xy
    fill = (24, 38, 50) if ok else (55, 30, 38)
    draw.rounded_rectangle((x, y, x + 196, y + 30), radius=15, fill=fill)
    draw.ellipse((x + 10, y + 10, x + 20, y + 20), fill=color if ok else BAD)
    draw.text((x + 28, y + 6), label, font=F14, fill=TEXT)


def render(args) -> None:
    sequence = args.root / "orbslam3-validation" / args.sequence
    compare = args.root / "rgbd-comparison" / args.sequence
    associations = [line.split() for line in (sequence / "associations.txt").read_text().splitlines() if line.strip()]
    orb_rows = read_csv(sequence / "baseline" / "tracking.csv")
    rtab_rows = read_csv(compare / "rtabmap-reset3s" / "frames.csv")
    count = min(len(associations), len(orb_rows), len(rtab_rows))
    if args.frames:
        count = min(count, args.frames)
    associations = associations[:count]
    orb_rows = orb_rows[:count]
    rtab_rows = rtab_rows[:count]

    orb_segments = split_segments(orb_rows, lambda r: r["state"] == "2", jump_limit=0.5)
    rtab_segments = split_segments(
        rtab_rows,
        lambda r: r["ok"] == "1",
        group=lambda r: int(r["map_epoch"]),
        jump_limit=0.5,
    )
    bounds = world_bounds(orb_segments, rtab_segments)
    timestamps = [float(row[0]) for row in associations]
    fps = (len(timestamps) - 1) / max(timestamps[-1] - timestamps[0], 0.001)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
        "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", f"{WIDTH}x{HEIGHT}",
        "-framerate", f"{fps:.6f}", "-i", "-", "-an", "-c:v", "libx264",
        "-preset", "medium", "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        str(args.output),
    ]
    ffmpeg = subprocess.Popen(command, stdin=subprocess.PIPE)
    assert ffmpeg.stdin is not None
    poster_index = min(args.poster_frame, count - 1)
    for index, (association, orb, rtab) in enumerate(zip(associations, orb_rows, rtab_rows)):
        _, rgb_name, _, depth_name = association
        canvas = Image.new("RGB", (WIDTH, HEIGHT), BG)
        draw = ImageDraw.Draw(canvas)
        draw.text((20, 18), "DABAI DC1  ·  MAPPING VISUAL VALIDATION", font=F24B, fill=TEXT)
        elapsed = float(association[0]) - timestamps[0]
        draw.text((930, 22), f"{elapsed:05.1f}s   frame {index + 1:04d}/{count}", font=F18, fill=MUTED)

        draw.rounded_rectangle(CAMERA_BOX, radius=14, fill=PANEL, outline=(38, 53, 70), width=2)
        camera = camera_frame(sequence / rgb_name, sequence / depth_name)
        mask = Image.new("L", camera.size, 0)
        ImageDraw.Draw(mask).rounded_rectangle((0, 0, camera.width, camera.height), radius=14, fill=255)
        canvas.paste(camera, (CAMERA_BOX[0], CAMERA_BOX[1]), mask)
        draw = ImageDraw.Draw(canvas)
        draw.rounded_rectangle((35, 94, 266, 126), radius=16, fill=(8, 15, 23))
        draw.text((49, 101), "RGB + 40% DEPTH OVERLAY", font=F14, fill=TEXT)
        crop_y = CAMERA_BOX[1] + round((CAMERA_BOX[3] - CAMERA_BOX[1]) * 400 / 480)
        draw.line((CAMERA_BOX[0], crop_y, CAMERA_BOX[2], crop_y), fill=(255, 255, 255), width=1)
        draw.text((CAMERA_BOX[0] + 12, crop_y - 24), "mapping crop boundary", font=F14, fill=(235, 240, 245))

        draw_grid(draw, bounds)
        orb_latest = draw_segments(draw, orb_segments, index, bounds, ORB, 4)
        rtab_latest = draw_segments(draw, rtab_segments, index, bounds, RTAB, 4)
        if orb_latest and orb["state"] == "2":
            draw.ellipse((orb_latest[0] - 7, orb_latest[1] - 7, orb_latest[0] + 7, orb_latest[1] + 7), fill=ORB, outline=TEXT, width=2)
        if rtab_latest and rtab["ok"] == "1":
            draw.rectangle((rtab_latest[0] - 6, rtab_latest[1] - 6, rtab_latest[0] + 6, rtab_latest[1] + 6), fill=RTAB, outline=TEXT, width=2)

        draw.rounded_rectangle(STATUS_BOX, radius=14, fill=PANEL, outline=(38, 53, 70), width=2)
        orb_ok = orb["state"] == "2"
        rtab_ok = rtab["ok"] == "1"
        status_pill(draw, (816, 571), f"ORB-SLAM3  {'TRACKING' if orb_ok else 'LOST'}", orb_ok, ORB)
        status_pill(draw, (1030, 571), f"RTAB-Map  {'TRACKING' if rtab_ok else 'LOST'}", rtab_ok, RTAB)
        draw.text((818, 612), f"ORB map points: {int(float(orb['map_points'])):4d}", font=F14, fill=MUTED)
        draw.text((1032, 612), f"RTAB epoch: {int(rtab['map_epoch'])}   inliers: {int(rtab['inliers'])}", font=F14, fill=MUTED)

        timeline_y = 682
        draw.line((20, timeline_y, 1260, timeline_y), fill=(50, 65, 82), width=5)
        progress_x = 20 + round(1240 * index / max(count - 1, 1))
        draw.line((20, timeline_y, progress_x, timeline_y), fill=(112, 147, 183), width=5)
        draw.ellipse((progress_x - 6, timeline_y - 6, progress_x + 6, timeline_y + 6), fill=TEXT)
        draw.text((20, 695), "Paths break on tracking loss; RTAB epochs are separate local maps.", font=F14, fill=MUTED)
        draw.text((832, 695), "Independent map frames · visual comparison, not metric alignment", font=F14, fill=MUTED)

        if index == poster_index:
            args.poster.parent.mkdir(parents=True, exist_ok=True)
            canvas.save(args.poster)
        ffmpeg.stdin.write(np.asarray(canvas, dtype=np.uint8).tobytes())

    ffmpeg.stdin.close()
    result = ffmpeg.wait()
    if result:
        raise SystemExit(f"ffmpeg failed with exit code {result}")
    print(f"video={args.output}")
    print(f"poster={args.poster}")
    print(f"frames={count} fps={fps:.3f} duration={count / fps:.2f}s")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root",
        type=Path,
        default=Path("artifacts/perception-dev/dabai-dc1"),
        help="DC1 artifact root",
    )
    parser.add_argument("--sequence", default="office-20260913-212016")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--poster", type=Path, required=True)
    parser.add_argument("--poster-frame", type=int, default=650)
    parser.add_argument("--frames", type=int, default=0, help="Limit frames for smoke tests")
    render(parser.parse_args())


if __name__ == "__main__":
    main()
