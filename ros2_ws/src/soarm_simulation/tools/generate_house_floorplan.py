#!/usr/bin/env python3
"""Generate a Gazebo model and occupancy map from the reference house.sh3d."""

from __future__ import annotations

import argparse
from collections import deque
import hashlib
import math
from pathlib import Path
import zipfile
from xml.etree import ElementTree as ET

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = PACKAGE_ROOT / "models" / "house_floorplan" / "model.sdf"
MAP_PATH = PACKAGE_ROOT / "maps" / "house_floorplan.pgm"
SOURCE_URL = "https://github.com/xtrinch/sweethome3d-house-plans.git"
SOURCE_COMMIT = "f0af4c6258ab5fcdabac1654e0796f9b18b44e94"
SOURCE_SHA256 = "1cc879ec4dbe435264a30973ec85ee5a0a221b314ebdf3c98680bf20cb6685e2"
EXPECTED_WALLS = 21
EXPECTED_DOORS = 10
WALL_HEIGHT = 2.4384
CENTER_X = 7.454154
CENTER_Y = 6.672447
MAP_RESOLUTION = 0.05
MAP_WIDTH = 320
MAP_HEIGHT = 290
MAP_ORIGIN_X = -8.0
MAP_ORIGIN_Y = -7.25
FRAME_WIDTH = 0.05

Wall = tuple[float, float, float, float, float]
Door = tuple[float, float, float, float, float]
WALLS: list[Wall] = []
DOORS: list[Door] = []


def load_geometry(source_path: Path) -> None:
    """Load the metric ground-floor structure directly from a Sweet Home 3D file."""
    global WALLS, DOORS
    source_digest = hashlib.sha256(source_path.read_bytes()).hexdigest()
    if source_digest != SOURCE_SHA256:
        raise ValueError(
            f"Expected house.sh3d SHA-256 {SOURCE_SHA256} from {SOURCE_COMMIT}, "
            f"got {source_digest}"
        )
    with zipfile.ZipFile(source_path) as archive:
        root = ET.fromstring(archive.read("Home.xml"))
    level = next(
        element.get("id") for element in root.findall("level")
        if element.get("name") == "Ground floor"
    )
    WALLS = [
        (
            float(element.get("xStart")) / 100,
            float(element.get("yStart")) / 100,
            float(element.get("xEnd")) / 100,
            float(element.get("yEnd")) / 100,
            float(element.get("thickness")) / 100,
        )
        for element in root.findall("wall")
        if element.get("level") == level
    ]
    DOORS = [
        (
            float(element.get("x")) / 100,
            float(element.get("y")) / 100,
            float(element.get("width")) / 100,
            float(element.get("height")) / 100,
            float(element.get("angle", "0")),
        )
        for element in root.findall("doorOrWindow")
        if element.get("level") == level
        and "door" in element.get("name", "").lower()
    ]
    if len(WALLS) != EXPECTED_WALLS or len(DOORS) != EXPECTED_DOORS:
        raise ValueError(
            f"Expected {EXPECTED_WALLS} walls and {EXPECTED_DOORS} door-like openings "
            f"from {SOURCE_COMMIT}, got {len(WALLS)} and {len(DOORS)}"
        )
    heights = {round(float(element.get("height")) / 100, 4)
               for element in root.findall("wall") if element.get("level") == level}
    if heights != {WALL_HEIGHT}:
        raise ValueError(f"Unexpected wall heights: {sorted(heights)}")


def plan_to_world(x: float, y: float) -> tuple[float, float]:
    return x - CENTER_X, CENTER_Y - y


def angle_difference(a: float, b: float) -> float:
    difference = (a - b) % math.pi
    return min(difference, math.pi - difference)


def matching_doors(wall: Wall) -> list[tuple[float, float, Door]]:
    x1, y1, x2, y2, thickness = wall
    dx, dy = x2 - x1, y2 - y1
    length = math.hypot(dx, dy)
    ux, uy = dx / length, dy / length
    wall_angle = math.atan2(dy, dx)
    matches = []
    for door in DOORS:
        x, y, width, _height, door_angle = door
        if angle_difference(wall_angle, door_angle) > 0.02:
            continue
        along = (x - x1) * ux + (y - y1) * uy
        perpendicular = abs((x - x1) * uy - (y - y1) * ux)
        opening_start = along - width / 2
        opening_end = along + width / 2
        if perpendicular <= thickness / 2 + 0.05 and opening_end > 0 and opening_start < length:
            matches.append((max(0.0, opening_start), min(length, opening_end), door))
    return sorted(matches)


def split_wall(wall: Wall) -> list[Wall]:
    x1, y1, x2, y2, thickness = wall
    dx, dy = x2 - x1, y2 - y1
    length = math.hypot(dx, dy)
    ux, uy = dx / length, dy / length
    sections = []
    cursor = 0.0
    for opening_start, opening_end, _door in matching_doors(wall):
        if opening_start > cursor + 0.01:
            sections.append((x1 + cursor * ux, y1 + cursor * uy,
                             x1 + opening_start * ux, y1 + opening_start * uy, thickness))
        cursor = max(cursor, opening_end)
    if cursor < length - 0.01:
        sections.append((x1 + cursor * ux, y1 + cursor * uy, x2, y2, thickness))
    return sections


def box_element(parent: ET.Element, kind: str, name: str, segment: Wall,
                height: float, z: float, color: str = "") -> None:
    x1, y1, x2, y2, thickness = segment
    world_x1, world_y1 = plan_to_world(x1, y1)
    world_x2, world_y2 = plan_to_world(x2, y2)
    length = math.hypot(world_x2 - world_x1, world_y2 - world_y1)
    yaw = math.atan2(world_y2 - world_y1, world_x2 - world_x1)
    element = ET.SubElement(parent, kind, {"name": name})
    ET.SubElement(element, "pose").text = (
        f"{(world_x1 + world_x2) / 2:.5f} {(world_y1 + world_y2) / 2:.5f} "
        f"{z:.5f} 0 0 {yaw:.7f}"
    )
    geometry = ET.SubElement(element, "geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = f"{length:.5f} {thickness:.5f} {height:.5f}"
    if kind == "visual":
        material = ET.SubElement(element, "material")
        ET.SubElement(material, "diffuse").text = color
        ET.SubElement(material, "ambient").text = color


def door_wall_thickness(door: Door) -> float:
    candidates = [wall[4] for wall in WALLS
                  if any(match[2] == door for match in matching_doors(wall))]
    if not candidates:
        raise ValueError(f"Door at {door[:2]} does not intersect a source wall")
    return max(candidates) + 0.02


def door_frame_segments(door: Door) -> list[Wall]:
    x, y, width, _opening_height, angle = door
    ux, uy = math.cos(angle), math.sin(angle)
    thickness = door_wall_thickness(door)
    segments = []
    # Jambs overlap the adjacent wall ends and preserve the source clear width.
    for offset in (-width / 2 - FRAME_WIDTH / 2, width / 2 + FRAME_WIDTH / 2):
        segments.append((
            x + (offset - FRAME_WIDTH / 2) * ux,
            y + (offset - FRAME_WIDTH / 2) * uy,
            x + (offset + FRAME_WIDTH / 2) * ux,
            y + (offset + FRAME_WIDTH / 2) * uy,
            thickness,
        ))
    return segments


def generate_sdf() -> None:
    sdf = ET.Element("sdf", {"version": "1.9"})
    model = ET.SubElement(sdf, "model", {"name": "house_floorplan"})
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", {"name": "structure"})

    wall_sections = [section for wall in WALLS for section in split_wall(wall)]
    for index, section in enumerate(wall_sections):
        box_element(link, "collision", f"wall_{index}", section, WALL_HEIGHT, WALL_HEIGHT / 2)
        box_element(link, "visual", f"wall_visual_{index}", section, WALL_HEIGHT,
                    WALL_HEIGHT / 2, "0.82 0.83 0.80 1")

    for index, door in enumerate(DOORS):
        x, y, width, opening_height, angle = door
        for side, segment in zip(("a", "b"), door_frame_segments(door)):
            box_element(link, "collision", f"door_frame_{index}_{side}", segment,
                        opening_height, opening_height / 2)
            box_element(link, "visual", f"door_frame_visual_{index}_{side}", segment,
                        opening_height, opening_height / 2, "0.45 0.31 0.18 1")
        ux, uy = math.cos(angle), math.sin(angle)
        header = (x - width / 2 * ux, y - width / 2 * uy,
                  x + width / 2 * ux, y + width / 2 * uy, door_wall_thickness(door))
        header_height = WALL_HEIGHT - opening_height
        box_element(link, "collision", f"door_header_{index}", header, header_height,
                    opening_height + header_height / 2)
        box_element(link, "visual", f"door_header_visual_{index}", header, header_height,
                    opening_height + header_height / 2, "0.45 0.31 0.18 1")

    ET.indent(sdf, space="  ")
    MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(sdf).write(MODEL_PATH, encoding="utf-8", xml_declaration=True)


def distance_to_segment(px: float, py: float, segment: Wall) -> float:
    x1, y1 = plan_to_world(segment[0], segment[1])
    x2, y2 = plan_to_world(segment[2], segment[3])
    dx, dy = x2 - x1, y2 - y1
    projection = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (x1 + projection * dx), py - (y1 + projection * dy))


def is_on_segment(world_x: float, world_y: float, segment: Wall) -> bool:
    return distance_to_segment(world_x, world_y, segment) <= segment[4] / 2


def generate_map() -> None:
    wall_sections = [section for wall in WALLS for section in split_wall(wall)]
    occupied_segments = wall_sections + [segment for door in DOORS for segment in door_frame_segments(door)]
    # These six source walls form the exterior L-shaped shell.
    shell_segments = WALLS[:5] + [WALLS[14]]
    shell = bytearray(MAP_WIDTH * MAP_HEIGHT)
    for row in range(MAP_HEIGHT):
        world_y = MAP_ORIGIN_Y + (MAP_HEIGHT - row - 0.5) * MAP_RESOLUTION
        for column in range(MAP_WIDTH):
            world_x = MAP_ORIGIN_X + (column + 0.5) * MAP_RESOLUTION
            shell[row * MAP_WIDTH + column] = any(
                is_on_segment(world_x, world_y, segment) for segment in shell_segments)

    outside = bytearray(MAP_WIDTH * MAP_HEIGHT)
    queue: deque[tuple[int, int]] = deque()
    for row in range(MAP_HEIGHT):
        for column in (0, MAP_WIDTH - 1):
            outside[row * MAP_WIDTH + column] = 1
            queue.append((row, column))
    for column in range(MAP_WIDTH):
        for row in (0, MAP_HEIGHT - 1):
            index = row * MAP_WIDTH + column
            if not outside[index]:
                outside[index] = 1
                queue.append((row, column))
    while queue:
        row, column = queue.popleft()
        for next_row, next_column in ((row - 1, column), (row + 1, column),
                                      (row, column - 1), (row, column + 1)):
            if not (0 <= next_row < MAP_HEIGHT and 0 <= next_column < MAP_WIDTH):
                continue
            index = next_row * MAP_WIDTH + next_column
            if outside[index] or shell[index]:
                continue
            outside[index] = 1
            queue.append((next_row, next_column))

    pixels = bytearray(205 if value else 254 for value in outside)
    for row in range(MAP_HEIGHT):
        world_y = MAP_ORIGIN_Y + (MAP_HEIGHT - row - 0.5) * MAP_RESOLUTION
        for column in range(MAP_WIDTH):
            world_x = MAP_ORIGIN_X + (column + 0.5) * MAP_RESOLUTION
            if any(is_on_segment(world_x, world_y, segment) for segment in occupied_segments):
                pixels[row * MAP_WIDTH + column] = 0

    MAP_PATH.parent.mkdir(parents=True, exist_ok=True)
    with MAP_PATH.open("wb") as output:
        output.write(f"P5\n{MAP_WIDTH} {MAP_HEIGHT}\n255\n".encode("ascii"))
        output.write(pixels)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_sh3d", type=Path, help=(
        "Path to house.sh3d checked out from " + SOURCE_URL + " at " + SOURCE_COMMIT
    ))
    arguments = parser.parse_args()
    load_geometry(arguments.source_sh3d)
    generate_sdf()
    generate_map()
    print(f"Generated {MODEL_PATH.relative_to(PACKAGE_ROOT)} from {arguments.source_sh3d}")
    print(f"Generated {MAP_PATH.relative_to(PACKAGE_ROOT)}")


if __name__ == "__main__":
    main()
