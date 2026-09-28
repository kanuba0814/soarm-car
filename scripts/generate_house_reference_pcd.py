#!/usr/bin/env python3
"""Generate a complete localization PCD from the house simulation geometry."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import struct
from xml.etree import ElementTree as ET

DEFAULT_SIMULATION_ROOT = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "soarm_simulation"
DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "artifacts" / "house_widened_reference.pcd"
DEFAULT_MAP_OUTPUT = DEFAULT_SIMULATION_ROOT / "maps" / "house_floorplan.pgm"
MAP_ORIGIN = (-8.0, -7.25)
MAP_RESOLUTION = 0.05
MAP_SIZE = (320, 290)
POINT_FORMAT = "<ffffffff"


def axis_samples(size: float, spacing: float) -> list[float]:
    count = max(1, math.ceil(size / spacing))
    return [-size / 2 + size * index / count for index in range(count + 1)]


def box_surface_points(
    pose: tuple[float, float, float, float],
    size: tuple[float, float, float],
    spacing: float,
) -> list[tuple[float, float, float, float, float, float, float, float]]:
    center_x, center_y, center_z, yaw = pose
    size_x, size_y, size_z = size
    cosine, sine = math.cos(yaw), math.sin(yaw)
    samples_x = axis_samples(size_x, spacing)
    samples_y = axis_samples(size_y, spacing)
    samples_z = axis_samples(size_z, spacing)
    points = []

    def add(local_x: float, local_y: float, local_z: float,
            normal_x: float, normal_y: float, normal_z: float) -> None:
        world_x = center_x + cosine * local_x - sine * local_y
        world_y = center_y + sine * local_x + cosine * local_y
        world_normal_x = cosine * normal_x - sine * normal_y
        world_normal_y = sine * normal_x + cosine * normal_y
        points.append((world_x, world_y, center_z + local_z, 0.0,
                       world_normal_x, world_normal_y, normal_z, 0.0))

    for local_y in samples_y:
        for local_z in samples_z:
            add(-size_x / 2, local_y, local_z, -1.0, 0.0, 0.0)
            add(size_x / 2, local_y, local_z, 1.0, 0.0, 0.0)
    for local_x in samples_x:
        for local_z in samples_z:
            add(local_x, -size_y / 2, local_z, 0.0, -1.0, 0.0)
            add(local_x, size_y / 2, local_z, 0.0, 1.0, 0.0)
    for local_x in samples_x:
        for local_y in samples_y:
            add(local_x, local_y, -size_z / 2, 0.0, 0.0, -1.0)
            add(local_x, local_y, size_z / 2, 0.0, 0.0, 1.0)
    return points


def load_collision_boxes(model_path: Path) -> list[tuple[tuple[float, ...], tuple[float, ...]]]:
    root = ET.parse(model_path).getroot()
    boxes = []
    for collision in root.findall(".//collision"):
        pose_values = tuple(float(value) for value in collision.findtext("pose").split())
        size_values = tuple(float(value) for value in collision.findtext("geometry/box/size").split())
        if len(pose_values) != 6 or len(size_values) != 3:
            raise ValueError(f"Unsupported collision geometry in {model_path}: {collision.get('name')}")
        boxes.append((pose_values, size_values))
    return boxes


def write_occupancy_map(output_path: Path, boxes: list[tuple[tuple[float, ...], tuple[float, ...]]]) -> None:
    width, height = MAP_SIZE
    pixels = bytearray([254]) * (width * height)
    for row in range(height):
        world_y = MAP_ORIGIN[1] + (height - row - 0.5) * MAP_RESOLUTION
        for column in range(width):
            world_x = MAP_ORIGIN[0] + (column + 0.5) * MAP_RESOLUTION
            for pose, size in boxes:
                cosine, sine = math.cos(pose[5]), math.sin(pose[5])
                delta_x, delta_y = world_x - pose[0], world_y - pose[1]
                local_x = cosine * delta_x + sine * delta_y
                local_y = -sine * delta_x + cosine * delta_y
                if abs(local_x) <= size[0] / 2 and abs(local_y) <= size[1] / 2:
                    pixels[row * width + column] = 0
                    break
    output_path.write_bytes(f"P5\n{width} {height}\n255\n".encode("ascii") + pixels)


def write_binary_pcd(output_path: Path, points: list[tuple[float, ...]]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity normal_x normal_y normal_z curvature\n"
        "SIZE 4 4 4 4 4 4 4 4\n"
        "TYPE F F F F F F F F\n"
        "COUNT 1 1 1 1 1 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    ).encode("ascii")
    with output_path.open("wb") as output:
        output.write(header)
        for point in points:
            output.write(struct.pack(POINT_FORMAT, *point))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulation-root", type=Path, default=DEFAULT_SIMULATION_ROOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--map-output", type=Path, default=DEFAULT_MAP_OUTPUT)
    parser.add_argument("--spacing", type=float, default=0.10)
    arguments = parser.parse_args()
    if arguments.spacing <= 0:
        parser.error("--spacing must be positive")

    model_path = arguments.simulation_root / "models" / "house_floorplan" / "model.sdf"
    boxes = load_collision_boxes(model_path)
    points = [
        point
        for pose, size in boxes
        for point in box_surface_points((pose[0], pose[1], pose[2], pose[5]), size, arguments.spacing)
    ]
    if not points:
        raise RuntimeError("No collision-box points were generated")
    write_binary_pcd(arguments.output, points)
    write_occupancy_map(arguments.map_output, boxes)
    print(f"Generated {len(points)} reference points and occupancy from {len(boxes)} SDF collisions")


if __name__ == "__main__":
    main()
