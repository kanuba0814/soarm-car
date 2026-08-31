#!/usr/bin/env python3
"""Generate the two explicitly selected local HM3D apartment runtime assets.

Run inside the project Docker image:
  python3 ros2_ws/src/soarm_simulation/tools/generate_hm3d_apartments.py \
    --dataset-root /datasets/hm3d-v0.2/val

The input mesh files and all output payloads derived from them are intentionally
ignored by Git.  This program is a fixed two-entry manifest, not a general
HM3D importer.
"""
import argparse
import csv
import hashlib
import heapq
import json
import math
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np
import open3d as o3d

SCRIPT = Path(__file__).resolve()
PACKAGE = SCRIPT.parents[1]
MANIFEST_PATH = PACKAGE / "config" / "hm3d_apartments.json"
STRUCTURAL = {"wall", "door frame", "column", "beam", "stairs"}
FLOORS = {"floor"}
FURNITURE_WORDS = (
    "bed", "sofa", "couch", "cabinet", "wardrobe", "table", "counter",
    "refrigerator", "desk", "shelf", "shelves", "dresser", "bookcase",
    "washer-dryer", "washing machine", "oven", "stove", "tv stand",
)
EXCLUDED_WORDS = ("curtain", "plant", "leaf", "lamp", "decoration")
DYNAMIC_WORDS = ("cup", "bottle", "remote", "book", "box", "fruit")


def linear_rgb(hex_color):
    """Convert annotation sRGB hex to Open3D's decoded linear vertex color."""
    srgb = np.array([int(hex_color[i:i + 2], 16) / 255.0 for i in (0, 2, 4)])
    return np.where(srgb <= 0.04045, srgb / 12.92, ((srgb + 0.055) / 1.055) ** 2.4)


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_bundle(paths, root):
    digest = hashlib.sha256()
    for path in sorted(paths):
        digest.update(str(path.relative_to(root)).encode("utf-8") + b"\0")
        digest.update(bytes.fromhex(sha256_file(path)))
    return digest.hexdigest()


def load_config():
    with MANIFEST_PATH.open() as stream:
        return json.load(stream)


def load_manifest():
    return load_config()["apartments"]


def metadata_path(dataset_root, metadata_root):
    root = metadata_root or dataset_root.parent / "metadata"
    path = root / "Per_Scene_Region_Weighted_Votes.csv"
    if not path.is_file():
        raise RuntimeError("missing room metadata CSV: " + str(path))
    return path


def load_room_categories(path, scene):
    categories = {}
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            if row["Scene Name"] == scene:
                categories[int(row["Region #"])] = row["Weighted Room Proposal"].strip()
    if not categories:
        raise RuntimeError("room metadata has no regions for " + scene)
    return categories


def parse_annotations(path):
    annotations = {}
    with path.open() as stream:
        for line in stream:
            if not line or line.startswith("HM3D"):
                continue
            fields = line.rstrip().split(",")
            if len(fields) != 4:
                continue
            instance_id, color, category, region_id = fields
            annotations[color.upper()] = {
                "instance_id": int(instance_id),
                "category": category.strip('"').lower(),
                "region_id": int(region_id),
                "color": color.upper(),
            }
    return annotations


def category_matches(category, words):
    return any(word in category for word in words)


def aabb(vertices):
    minimum = vertices.min(axis=0)
    maximum = vertices.max(axis=0)
    size = maximum - minimum
    return {
        "min": minimum.round(6).tolist(), "max": maximum.round(6).tolist(),
        "center": ((minimum + maximum) / 2).round(6).tolist(),
        "size": size.round(6).tolist(),
    }


def write_visual(source, mesh_dir):
    """Expand an embedded-texture GLB into Fortress-compatible OBJ assets."""
    assimp = shutil.which("assimp")
    if not assimp:
        raise RuntimeError("assimp executable is required (install assimp-utils)")
    mesh_dir.mkdir(parents=True, exist_ok=True)
    for path in mesh_dir.glob("visual*"):
        if path.is_file() or path.is_symlink():
            path.unlink()
    with tempfile.TemporaryDirectory(prefix="hm3d_assimp_") as temporary:
        temporary = Path(temporary)
        result = subprocess.run(
            [assimp, "export", str(source), str(temporary / "visual.obj")],
            capture_output=True, text=True)
        if result.returncode or not (temporary / "visual.obj").is_file() or not (temporary / "visual.mtl").is_file():
            raise RuntimeError("assimp OBJ export failed: " + (result.stderr or result.stdout).strip())
        material = (temporary / "visual.mtl").read_text()
        texture_indices = sorted({int(value) for value in re.findall(r"^map_Kd \*(\d+)$", material, re.MULTILINE)})
        if not texture_indices:
            raise RuntimeError("assimp OBJ export did not preserve embedded texture references")
        replacements = {}
        for index in texture_indices:
            prefix = temporary / ("texture_%03d" % index)
            subprocess.run(
                [assimp, "extract", str(source), str(prefix), "-t%d" % index],
                capture_output=True, text=True)
            candidates = sorted(temporary.glob(prefix.name + "*"))
            candidates = [path for path in candidates if path.is_file()]
            if len(candidates) != 1:
                raise RuntimeError("failed to extract embedded visual texture %d" % index)
            suffix = candidates[0].suffix.lower()
            if suffix not in {".jpg", ".jpeg", ".png"}:
                raise RuntimeError("unsupported extracted texture format: " + suffix)
            target_name = "visual_texture_%03d%s" % (index, suffix)
            shutil.move(str(candidates[0]), mesh_dir / target_name)
            replacements[index] = target_name
        material = re.sub(
            r"^map_Kd \*(\d+)$",
            lambda match: "map_Kd " + replacements[int(match.group(1))],
            material,
            flags=re.MULTILINE)
        shutil.move(str(temporary / "visual.obj"), mesh_dir / "visual.obj")
        (mesh_dir / "visual.mtl").write_text(material)


def mesh_for_triangles(vertices, triangles):
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(triangles)
    return mesh


def collect_instances(semantic_mesh, annotations):
    vertices = np.asarray(semantic_mesh.vertices)
    triangles = np.asarray(semantic_mesh.triangles)
    colors = np.asarray(semantic_mesh.vertex_colors)
    if not len(vertices) or not len(triangles) or len(colors) != len(vertices):
        raise RuntimeError("semantic GLB has no complete vertex/triangle/color data")
    expected = {key: linear_rgb(key) for key in annotations}
    instance_triangles = defaultdict(list)
    # Every semantic triangle should use a single instance color. Ignore black
    # (documented unlabelled geometry) and non-uniform/unknown triangles.
    for triangle in triangles:
        triangle_colors = colors[triangle]
        if not np.allclose(triangle_colors[0], triangle_colors[1:], atol=0.004):
            continue
        color = triangle_colors[0]
        if np.max(color) < 0.001:
            continue
        nearest = min(expected, key=lambda key: float(np.linalg.norm(expected[key] - color)))
        if np.linalg.norm(expected[nearest] - color) <= 0.015:
            instance_triangles[nearest].append(triangle)
    records = []
    for color, annotation in annotations.items():
        selected = instance_triangles.get(color, [])
        record = dict(annotation)
        record["dynamic_model_required"] = category_matches(annotation["category"], DYNAMIC_WORDS)
        record["excluded_from_static_collision"] = record["dynamic_model_required"] or category_matches(annotation["category"], EXCLUDED_WORDS)
        if selected:
            indices = np.unique(np.asarray(selected).reshape(-1))
            record["aabb"] = aabb(vertices[indices])
        records.append(record)
    return records, instance_triangles


def simplify_structural(vertices, instance_triangles, annotations, destination):
    selected = []
    for color, triangles in instance_triangles.items():
        if annotations[color]["category"] in STRUCTURAL:
            selected.extend(triangles)
    if not selected:
        raise RuntimeError("semantic color matching found no structural triangles")
    mesh = mesh_for_triangles(vertices, np.asarray(selected))
    mesh.remove_duplicated_vertices()
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    original = len(mesh.triangles)
    target = min(12000, max(2000, original // 12))
    if original > target:
        mesh = mesh.simplify_quadric_decimation(target)
    mesh.compute_vertex_normals()
    if not o3d.io.write_triangle_mesh(str(destination), mesh, write_vertex_normals=True):
        raise RuntimeError("failed to write structural collision OBJ")
    return original, len(mesh.triangles)


def collision_boxes(records):
    boxes = []
    for record in records:
        category = record["category"]
        if "aabb" not in record or record["excluded_from_static_collision"]:
            continue
        if category in FLOORS or category_matches(category, FURNITURE_WORDS):
            size = np.maximum(np.asarray(record["aabb"]["size"]), 0.05)
            # A giant annotation is normally room-level architecture, not furniture.
            if category not in FLOORS and np.prod(size) > 100.0:
                continue
            boxes.append((record["instance_id"], category, record["aabb"]["center"], size.tolist()))
    return boxes


def sdf_box(name, center, size):
    return """      <collision name=\"%s\"><pose>%.6f %.6f %.6f 0 0 0</pose><geometry><box><size>%.6f %.6f %.6f</size></box></geometry></collision>""" % (
        name, center[0], center[1], center[2], size[0], size[1], size[2])


def write_model_sdf(apartment, boxes):
    name = apartment["name"]
    collision_lines = [
        "      <collision name=\"structural_collision\"><geometry><mesh><uri>meshes/structural_collision.obj</uri></mesh></geometry></collision>"
    ]
    collision_lines.extend(sdf_box("box_%d" % instance_id, center, size) for instance_id, _, center, size in boxes)
    text = """<?xml version=\"1.0\"?>
<sdf version=\"1.9\"><model name=\"%s\"><static>true</static><link name=\"apartment\">
%s
      <visual name=\"hm3d_visual\"><geometry><mesh><uri>meshes/visual.obj</uri></mesh></geometry></visual>
    </link></model></sdf>
""" % (name, "\n".join(collision_lines))
    (PACKAGE / "models" / name / "model.sdf").write_text(text)


def cell_range(minimum, maximum, origin, resolution, limit):
    return max(0, int(math.floor((minimum - origin) / resolution))), min(limit - 1, int(math.ceil((maximum - origin) / resolution)))


def rasterize_triangle(mask, triangle, origin, resolution):
    xy = triangle[:, :2]
    minimum, maximum = xy.min(axis=0), xy.max(axis=0)
    x0, x1 = cell_range(minimum[0], maximum[0], origin[0], resolution, mask.shape[1])
    y0, y1 = cell_range(minimum[1], maximum[1], origin[1], resolution, mask.shape[0])
    area = np.cross(xy[1] - xy[0], xy[2] - xy[0])
    if abs(area) < 1e-9:
        # Vertical structural faces project to a line; retain each vertex cell.
        for point in xy:
            x = int(math.floor((point[0] - origin[0]) / resolution))
            y = int(math.floor((point[1] - origin[1]) / resolution))
            if 0 <= x < mask.shape[1] and 0 <= y < mask.shape[0]:
                mask[y, x] = True
        return
    xs, ys = np.meshgrid(np.arange(x0, x1 + 1), np.arange(y0, y1 + 1))
    points = np.stack((origin[0] + (xs + 0.5) * resolution, origin[1] + (ys + 0.5) * resolution), axis=-1)
    cross_a = np.cross(xy[1] - xy[0], points - xy[0])
    cross_b = np.cross(xy[2] - xy[1], points - xy[1])
    cross_c = np.cross(xy[0] - xy[2], points - xy[2])
    inside = ((cross_a >= -1e-8) & (cross_b >= -1e-8) & (cross_c >= -1e-8)) | ((cross_a <= 1e-8) & (cross_b <= 1e-8) & (cross_c <= 1e-8))
    mask[y0:y1 + 1, x0:x1 + 1] |= inside


def dilate(mask, radius):
    result = mask.copy()
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            source_y0, source_y1 = max(0, -dy), min(mask.shape[0], mask.shape[0] - dy)
            source_x0, source_x1 = max(0, -dx), min(mask.shape[1], mask.shape[1] - dx)
            result[source_y0 + dy:source_y1 + dy, source_x0 + dx:source_x1 + dx] |= mask[source_y0:source_y1, source_x0:source_x1]
    return result


def clearance_grid(free, occupied, resolution):
    distance = np.full(free.shape, np.inf)
    queue = []
    for y, x in np.argwhere(occupied):
        distance[y, x] = 0.0
        heapq.heappush(queue, (0.0, int(y), int(x)))
    for y, x in np.argwhere(~free):
        if distance[y, x] > 0.0:
            distance[y, x] = 0.0
            heapq.heappush(queue, (0.0, int(y), int(x)))
    for distance_value, y, x in iter(lambda: heapq.heappop(queue) if queue else None, None):
        if distance_value != distance[y, x]:
            continue
        for dy, dx, cost in ((-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1), (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))):
            ny, nx = y + dy, x + dx
            candidate = distance_value + cost * resolution
            if 0 <= ny < free.shape[0] and 0 <= nx < free.shape[1] and candidate < distance[ny, nx]:
                distance[ny, nx] = candidate
                heapq.heappush(queue, (candidate, ny, nx))
    return distance


def pick_spawn(map_data):
    free, clearance, origin, resolution, floor_top = map_data["free"], map_data["clearance"], map_data["origin"], map_data["resolution"], map_data["floor_top"]
    candidates = np.argwhere(free & (clearance >= 0.55))
    if not len(candidates):
        raise RuntimeError("cannot derive spawn: no floor cell has 0.55 m clearance")
    y, x = candidates[np.argmax(clearance[free & (clearance >= 0.55)])]
    return [float(origin[0] + (x + 0.5) * resolution), float(origin[1] + (y + 0.5) * resolution), float(floor_top[y, x] + 0.08)], float(clearance[y, x]), (int(y), int(x))


def write_world(apartment, spawn):
    text = """<?xml version=\"1.0\"?>
<sdf version=\"1.9\"><world name=\"%s\">
  <physics name=\"1ms\" type=\"dart\"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
  <plugin filename=\"ignition-gazebo-physics-system\" name=\"ignition::gazebo::systems::Physics\"/>
  <plugin filename=\"ignition-gazebo-user-commands-system\" name=\"ignition::gazebo::systems::UserCommands\"/>
  <plugin filename=\"ignition-gazebo-scene-broadcaster-system\" name=\"ignition::gazebo::systems::SceneBroadcaster\"/>
  <plugin filename=\"ignition-gazebo-sensors-system\" name=\"ignition::gazebo::systems::Sensors\"><render_engine>ogre2</render_engine></plugin>
  <plugin filename=\"ignition-gazebo-imu-system\" name=\"ignition::gazebo::systems::Imu\"/>
  <scene><ambient>0.12 0.12 0.12 1</ambient><background>0.18 0.18 0.18 1</background><shadows>true</shadows></scene>
  <light type=\"directional\" name=\"sun\"><pose>0 0 10 0 0 0</pose><cast_shadows>true</cast_shadows><direction>-0.5 0.1 -0.9</direction><diffuse>0.22 0.22 0.22 1</diffuse><specular>0.05 0.05 0.05 1</specular></light>
  <light type=\"point\" name=\"fill\"><pose>0 0 4 0 0 0</pose><diffuse>0.3 0.3 0.3 1</diffuse><specular>0.02 0.02 0.02 1</specular><intensity>1.5</intensity><attenuation><range>30</range><linear>0.05</linear><quadratic>0.01</quadratic></attenuation></light>
  <include><uri>model://%s</uri></include>
  <include><uri>model://senior_akm</uri><name>senior_akm</name><pose>%.6f %.6f %.6f 0 0 %.6f</pose></include>
</world></sdf>
""" % (apartment["name"], apartment["name"], spawn[0], spawn[1], spawn[2], apartment["regression_yaw"])
    (PACKAGE / "worlds" / (apartment["name"] + ".sdf")).write_text(text)


def write_map(apartment, records, boxes, vertices, instance_triangles, annotations):
    floors = [record for record in records if record["category"] in FLOORS and "aabb" in record]
    if not floors:
        raise RuntimeError("cannot rasterize map: no labelled floor regions")
    mins = np.min(np.array([record["aabb"]["min"] for record in floors]), axis=0)
    maxs = np.max(np.array([record["aabb"]["max"] for record in floors]), axis=0)
    resolution, margin = 0.10, 0.8
    origin = np.array([mins[0] - margin, mins[1] - margin])
    width = int(math.ceil((maxs[0] - mins[0] + 2 * margin) / resolution))
    height = int(math.ceil((maxs[1] - mins[1] + 2 * margin) / resolution))
    floor_mask = np.zeros((height, width), dtype=bool)
    floor_top = np.full((height, width), np.nan)
    structural_mask = np.zeros((height, width), dtype=bool)
    for color, triangles in instance_triangles.items():
        category = annotations[color]["category"]
        for indices in triangles:
            triangle = vertices[indices]
            if category in FLOORS:
                triangle_mask = np.zeros_like(floor_mask)
                rasterize_triangle(triangle_mask, triangle, origin, resolution)
                floor_mask |= triangle_mask
                top = float(np.max(triangle[:, 2]))
                floor_top[triangle_mask] = np.where(np.isnan(floor_top[triangle_mask]), top, np.maximum(floor_top[triangle_mask], top))
            # Map only geometry that can obstruct the robot body. Door headers
            # above this band are deliberately excluded, preserving openings.
            elif category in STRUCTURAL and np.min(triangle[:, 2]) <= 0.75 and np.max(triangle[:, 2]) >= 0.05:
                rasterize_triangle(structural_mask, triangle, origin, resolution)
    if not floor_mask.any():
        raise RuntimeError("floor semantic geometry did not rasterize")
    occupied = dilate(structural_mask, 1)
    for _, category, center, size in boxes:
        if category in FLOORS:
            continue
        x0, x1 = cell_range(center[0] - size[0] / 2, center[0] + size[0] / 2, origin[0], resolution, width)
        y0, y1 = cell_range(center[1] - size[1] / 2, center[1] + size[1] / 2, origin[1], resolution, height)
        occupied[y0:y1 + 1, x0:x1 + 1] = True
    free = floor_mask & ~occupied
    floor_top[~free] = np.nan
    clearance = clearance_grid(free, occupied, resolution)
    image = np.full((height, width), 205, dtype=np.uint8)
    image[free] = 254
    image[occupied & floor_mask] = 0
    name = apartment["name"]
    with (PACKAGE / "maps" / (name + ".pgm")).open("wb") as stream:
        stream.write(("P5\n%d %d\n255\n" % (width, height)).encode("ascii"))
        stream.write(np.flipud(image).tobytes())
    (PACKAGE / "maps" / (name + ".yaml")).write_text(
        "image: %s.pgm\nmode: trinary\nresolution: %.3f\norigin: [%.6f, %.6f, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n" % (name, resolution, origin[0], origin[1]))
    structural_occupied = structural_mask & floor_mask
    return {"free": free, "occupied": occupied & floor_mask, "clearance": clearance, "origin": origin, "resolution": resolution, "floor_top": floor_top,
            "counts": {"free": int(free.sum()), "occupied": int((occupied & floor_mask).sum()), "unknown": int((~floor_mask).sum()), "structural_occupied": int(structural_occupied.sum())}}


def generate(apartment, dataset_root, metadata_root):
    source_dir = dataset_root / apartment["scene"]
    stem = apartment["stem"]
    visual, semantic, annotation = (source_dir / (stem + suffix) for suffix in (".glb", ".semantic.glb", ".semantic.txt"))
    missing = [str(path) for path in (visual, semantic, annotation) if not path.is_file()]
    if missing:
        raise RuntimeError("missing source files: " + ", ".join(missing))
    room_metadata = metadata_path(dataset_root, metadata_root)
    annotations = parse_annotations(annotation)
    room_categories = load_room_categories(room_metadata, apartment["scene"])
    semantic_mesh = o3d.io.read_triangle_mesh(str(semantic), enable_post_processing=False)
    records, triangles = collect_instances(semantic_mesh, annotations)
    vertices = np.asarray(semantic_mesh.vertices)
    mesh_dir = PACKAGE / "models" / apartment["name"] / "meshes"
    write_visual(visual, mesh_dir)
    original_triangles, collision_triangles = simplify_structural(vertices, triangles, annotations, mesh_dir / "structural_collision.obj")
    boxes = collision_boxes(records)
    if not boxes:
        raise RuntimeError("no floor or major furniture collision boxes were generated")
    room_ids = sorted({record["region_id"] for record in records})
    missing_categories = [region_id for region_id in room_ids if region_id not in room_categories]
    if missing_categories:
        raise RuntimeError("room metadata missing semantic regions for %s: %s" % (apartment["name"], missing_categories))
    rooms = [{"room_id": region_id, "category": room_categories[region_id], "region_id": region_id} for region_id in room_ids]
    map_data = write_map(apartment, records, boxes, vertices, triangles, annotations)
    spawn, clearance, spawn_cell = pick_spawn(map_data)
    write_model_sdf(apartment, boxes)
    write_world(apartment, spawn)
    model_path = PACKAGE / "models" / apartment["name"] / "model.sdf"
    world_path = PACKAGE / "worlds" / (apartment["name"] + ".sdf")
    map_path = PACKAGE / "maps" / (apartment["name"] + ".pgm")
    map_yaml_path = PACKAGE / "maps" / (apartment["name"] + ".yaml")
    output = {"source_scene": apartment["scene"], "coordinate_frame": "HM3D metric Z-up", "rooms": rooms, "objects": records,
              "collision": {"structural_source_triangles": original_triangles, "structural_triangles": collision_triangles, "box_count": len(boxes)},
              "navigation": {"spawn": spawn, "spawn_clearance_m": round(clearance, 6), "spawn_cell": [spawn_cell[1], spawn_cell[0]], "map_counts": map_data["counts"],
                             "regression_camera": {"robot_pose": spawn + [0.0, 0.0, apartment["regression_yaw"]], "topic": "/camera/image", "lighting_profile": load_config()["lighting_profile"]}},
              "provenance": {"sha256": {
                  "source_visual_glb": sha256_file(visual),
                  "source_semantic_glb": sha256_file(semantic),
                  "source_semantic_txt": sha256_file(annotation),
                  "room_metadata_csv": sha256_file(room_metadata),
                  "visual_bundle": sha256_bundle(mesh_dir.glob("visual*"), mesh_dir),
                  "structural_collision_obj": sha256_file(mesh_dir / "structural_collision.obj"),
                  "model_sdf": sha256_file(model_path),
                  "world_sdf": sha256_file(world_path),
                  "occupancy_pgm": sha256_file(map_path),
                  "occupancy_yaml": sha256_file(map_yaml_path),
              }}}
    (PACKAGE / "semantics" / (apartment["name"] + ".json")).write_text(json.dumps(output, indent=2) + "\n")
    print("generated %s: %d objects, %d rooms, %d structural triangles, %d boxes, %.2f m spawn clearance" % (apartment["name"], len(records), len(rooms), collision_triangles, len(boxes), clearance))


def read_pgm(path):
    with path.open("rb") as stream:
        if stream.readline().strip() != b"P5":
            raise RuntimeError("map is not binary PGM: " + str(path))
        dimensions = stream.readline().split()
        width, height = int(dimensions[0]), int(dimensions[1])
        if int(stream.readline()) != 255:
            raise RuntimeError("map has unsupported max value: " + str(path))
        image = np.frombuffer(stream.read(), dtype=np.uint8)
    if image.size != width * height:
        raise RuntimeError("map has incomplete pixel data: " + str(path))
    return np.flipud(image.reshape((height, width)))


def validate(apartment, dataset_root, metadata_root):
    source_dir = dataset_root / apartment["scene"]
    stem = apartment["stem"]
    expected_sources = [source_dir / (stem + suffix) for suffix in (".glb", ".semantic.glb", ".semantic.txt")]
    if not all(path.is_file() for path in expected_sources):
        raise RuntimeError("missing source files for " + apartment["name"])
    room_metadata = metadata_path(dataset_root, metadata_root)
    room_categories = load_room_categories(room_metadata, apartment["scene"])
    mesh_dir = PACKAGE / "models" / apartment["name"] / "meshes"
    outputs = [mesh_dir / "visual.obj", mesh_dir / "structural_collision.obj", PACKAGE / "models" / apartment["name"] / "model.sdf", PACKAGE / "worlds" / (apartment["name"] + ".sdf"), PACKAGE / "maps" / (apartment["name"] + ".pgm"), PACKAGE / "maps" / (apartment["name"] + ".yaml"), PACKAGE / "semantics" / (apartment["name"] + ".json")]
    if not all(path.is_file() and path.stat().st_size > 0 for path in outputs):
        raise RuntimeError("missing generated outputs for " + apartment["name"])
    data = json.loads(outputs[-1].read_text())
    if data.get("source_scene") != apartment["scene"]:
        raise RuntimeError("semantic source scene mismatch for " + apartment["name"])
    provenance = data.get("provenance", {}).get("sha256", {})
    expected_hashes = {
        "source_visual_glb": sha256_file(expected_sources[0]),
        "source_semantic_glb": sha256_file(expected_sources[1]),
        "source_semantic_txt": sha256_file(expected_sources[2]),
        "room_metadata_csv": sha256_file(room_metadata),
        "visual_bundle": sha256_bundle(mesh_dir.glob("visual*"), mesh_dir),
        "structural_collision_obj": sha256_file(outputs[1]),
        "model_sdf": sha256_file(outputs[2]),
        "world_sdf": sha256_file(outputs[3]),
        "occupancy_pgm": sha256_file(outputs[4]),
        "occupancy_yaml": sha256_file(outputs[5]),
    }
    if provenance != expected_hashes:
        mismatches = sorted(key for key, value in expected_hashes.items() if provenance.get(key) != value)
        raise RuntimeError("source/generated provenance hash mismatch for %s: %s" % (apartment["name"], ", ".join(mismatches)))
    if not data.get("rooms") or len(data.get("objects", [])) < 100:
        raise RuntimeError("semantic output has empty rooms or too few objects for " + apartment["name"])
    if all(room.get("category", "").strip().lower() in {"unknown", "unknown room"} for room in data["rooms"]):
        raise RuntimeError("room output has only unknown categories for " + apartment["name"])
    for room in data["rooms"]:
        if room["region_id"] not in room_categories or room["category"] != room_categories[room["region_id"]]:
            raise RuntimeError("room metadata category mismatch for " + apartment["name"])
    collision = data.get("collision", {})
    if collision.get("structural_triangles", 0) >= collision.get("structural_source_triangles", 0) or collision.get("box_count", 0) == 0:
        raise RuntimeError("collision is not simplified structural mesh plus boxes for " + apartment["name"])
    model = ET.parse(outputs[2]).getroot()
    visuals = model.findall(".//visual/geometry/mesh/uri")
    collisions = model.findall(".//collision/geometry/mesh/uri")
    if len(visuals) != 1 or visuals[0].text != "meshes/visual.obj" or any(uri.text == "meshes/visual.obj" for uri in collisions):
        raise RuntimeError("model SDF visual/collision mesh contract failed for " + apartment["name"])
    material_path = mesh_dir / "visual.mtl"
    if not material_path.is_file():
        raise RuntimeError("visual OBJ is missing its material library for " + apartment["name"])
    texture_references = re.findall(r"^map_Kd (.+)$", material_path.read_text(), re.MULTILINE)
    if not texture_references or any(reference.startswith("*") or not (mesh_dir / reference).is_file() for reference in texture_references):
        raise RuntimeError("visual material has missing embedded textures for " + apartment["name"])
    if any("http://" in path.read_text(errors="ignore") or "https://" in path.read_text(errors="ignore") for path in outputs[2:4]):
        raise RuntimeError("online URI found for " + apartment["name"])
    image = read_pgm(outputs[4])
    free_count, occupied_count, unknown_count = (int((image == value).sum()) for value in (254, 0, 205))
    if not free_count or not occupied_count or not unknown_count:
        raise RuntimeError("map lacks meaningful free, occupied, or unknown populations for " + apartment["name"])
    if data.get("navigation", {}).get("map_counts", {}).get("structural_occupied", 0) <= 0:
        raise RuntimeError("map lacks rasterized structural obstacles for " + apartment["name"])
    navigation = data.get("navigation", {})
    spawn = navigation.get("spawn", [])
    if len(spawn) != 3 or navigation.get("spawn_clearance_m", 0.0) < 0.55:
        raise RuntimeError("spawn lacks required grid clearance for " + apartment["name"])
    yaml_values = dict(line.split(":", 1) for line in outputs[5].read_text().splitlines() if ":" in line)
    resolution = float(yaml_values["resolution"].strip())
    origin = [float(value) for value in yaml_values["origin"].strip().strip("[]").split(",")[:2]]
    x, y = int(math.floor((spawn[0] - origin[0]) / resolution)), int(math.floor((spawn[1] - origin[1]) / resolution))
    if not (0 <= y < image.shape[0] and 0 <= x < image.shape[1] and image[y, x] == 254):
        raise RuntimeError("spawn is not in a free map cell for " + apartment["name"])
    world = ET.parse(outputs[3]).getroot()
    robot_pose = world.find(".//include[name='senior_akm']/pose")
    expected_pose = spawn + [0.0, 0.0, apartment["regression_yaw"]]
    regression_camera = navigation.get("regression_camera", {})
    if regression_camera.get("robot_pose") != expected_pose or regression_camera.get("topic") != "/camera/image" or regression_camera.get("lighting_profile") != load_config().get("lighting_profile"):
        raise RuntimeError("regression camera checkpoint mismatch for " + apartment["name"])
    if robot_pose is None or not np.allclose([float(value) for value in robot_pose.text.split()[:6]], expected_pose, atol=1e-6):
        raise RuntimeError("world spawn does not match validated regression pose for " + apartment["name"])
    print("validated %s: %d objects, %d rooms, map free/occupied/unknown=%d/%d/%d, spawn clearance %.2f m" % (apartment["name"], len(data["objects"]), len(data["rooms"]), free_count, occupied_count, unknown_count, navigation["spawn_clearance_m"]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--metadata-root", type=Path, help="Directory containing Per_Scene_Region_Weighted_Votes.csv; defaults to the dataset-root sibling metadata directory.")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    try:
        for apartment in load_manifest():
            if args.validate_only:
                validate(apartment, args.dataset_root, args.metadata_root)
            else:
                generate(apartment, args.dataset_root, args.metadata_root)
    except Exception as error:
        print("HM3D apartment generation failed: " + str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
