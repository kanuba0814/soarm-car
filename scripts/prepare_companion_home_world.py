#!/usr/bin/env python3
"""Create an isolated runtime world and optional evaluator-only segmentation overlay."""
import hashlib
import json
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

source, destination, *options = sys.argv[1:]
valid = {"--metric-calibration-panel", "--evaluator-segmentation", "--segmentation-occlusion-smoke"}
if any(option not in valid for option in options):
    raise SystemExit("unsupported runtime world option")
options = set(options)
if "--segmentation-occlusion-smoke" in options:
    options.add("--evaluator-segmentation")
tree = ET.parse(source)
world = tree.getroot().find("world")
if world is None:
    raise SystemExit("Pinned world has no world element")
world.set("name", "default")

label_map = []
if "--evaluator-segmentation" in options:
    # Source world wraps each physical include in a model. Flatten only the generated
    # wrapper so the official Label plugin attaches directly to each world include.
    source_models = list(world.findall("model"))
    for label, model in enumerate(source_models, start=1):
        name = model.get("name")
        uri = model.findtext("include/uri")
        pose = model.findtext("pose", default="")
        if not name or not uri:
            raise SystemExit("source model lacks stable name or include uri")
        include = ET.Element("include")
        ET.SubElement(include, "uri").text = uri
        ET.SubElement(include, "name").text = name
        ET.SubElement(include, "pose").text = pose
        plugin = ET.SubElement(include, "plugin", {"filename": "ignition-gazebo-label-system", "name": "ignition::gazebo::systems::Label"})
        ET.SubElement(plugin, "label").text = str(label)
        world.insert(list(world).index(model), include)
        world.remove(model)
        label_map.append({"label_id": label, "source_name": name, "source_uri": uri, "source_pose": pose})
    if len(label_map) > 255:
        raise SystemExit("semantic label range exceeds 8-bit evaluator contract")

robot = ET.SubElement(world, "include")
ET.SubElement(robot, "uri").text = "model://kuruma_eval" if "--evaluator-segmentation" in options else "model://kuruma"
ET.SubElement(robot, "name").text = "kuruma"
ET.SubElement(robot, "pose").text = "0 0 0.08 0 0 0"

if "--evaluator-segmentation" in options:
    native_model = Path("/tmp/companion/install/soarm_simulation/share/soarm_simulation/models/kuruma")
    runtime_root = Path("/tmp/companion-evaluator-models")
    generated = runtime_root / "kuruma_eval"
    shutil.rmtree(generated, ignore_errors=True)
    shutil.copytree(native_model, generated)
    model_path = generated / "model.sdf"
    model_tree = ET.parse(model_path)
    camera_link = model_tree.getroot().find("model/link[@name='camera_link']")
    if camera_link is None:
        raise SystemExit("native Kuruma camera_link missing")
    sensor = ET.SubElement(camera_link, "sensor", {"name": "eval_segmentation", "type": "segmentation"})
    ET.SubElement(sensor, "topic").text = "/eval_segmentation"
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = "15"
    camera = ET.SubElement(sensor, "camera")
    ET.SubElement(camera, "segmentation_type").text = "semantic"
    ET.SubElement(camera, "horizontal_fov").text = "1.047"
    image = ET.SubElement(camera, "image")
    ET.SubElement(image, "width").text = "640"
    ET.SubElement(image, "height").text = "480"
    clip = ET.SubElement(camera, "clip")
    ET.SubElement(clip, "near").text = "0.1"
    ET.SubElement(clip, "far").text = "8"
    model_tree.write(model_path, encoding="utf-8", xml_declaration=True)
    label_path = Path("/output/private_instance_labels.json")
    label_path.write_text(json.dumps({"scope": "EVALUATOR_PRIVATE_ONLY", "labels": label_map}, indent=2) + "\n")
    Path("/output/evaluator_runtime_overlay.json").write_text(json.dumps({"scope": "EVALUATOR_PRIVATE_ONLY", "options": sorted(options), "native_kuruma_model_sha256": hashlib.sha256((native_model / "model.sdf").read_bytes()).hexdigest(), "generated_kuruma_eval_model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(), "private_label_map_sha256": hashlib.sha256(label_path.read_bytes()).hexdigest(), "label_count": len(label_map), "sensor": {"type": "segmentation", "topic": "/eval_segmentation", "mode": "semantic", "width": 640, "height": 480, "fov": 1.047, "near": 0.1, "far": 8.0, "update_rate": 15}}, indent=2) + "\n")

if "--segmentation-occlusion-smoke" in options:
    for name, pose, label, color in (("eval_rear", "-5.0 0.9 0.5 0 0 0", 250, "1 0 0 1"), ("eval_front", "-5.8 0.9 0.5 0 0 0", 251, "0 1 0 1")):
        model = ET.SubElement(world, "model", {"name": name})
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = pose
        plugin = ET.SubElement(model, "plugin", {"filename": "ignition-gazebo-label-system", "name": "ignition::gazebo::systems::Label"})
        ET.SubElement(plugin, "label").text = str(label)
        link = ET.SubElement(model, "link", {"name": "link"})
        visual = ET.SubElement(link, "visual", {"name": "visual"})
        geometry = ET.SubElement(visual, "geometry"); box = ET.SubElement(geometry, "box"); ET.SubElement(box, "size").text = "0.1 1.2 1.2"
        material = ET.SubElement(visual, "material"); ET.SubElement(material, "diffuse").text = color

if "--metric-calibration-panel" in options:
    for name, pose, size, color in (("metric_calibration_panel", "-5.0 0.9 0.5 0 0 0", "0.02 2.0 2.0", "0 1 0 1"), ("metric_calibration_panel_y", "-6.9 2.8 0.5 0 0 0", "2.0 0.02 2.0", "0 0 1 1")):
        model = ET.SubElement(world, "model", {"name": name}); ET.SubElement(model, "static").text = "true"; ET.SubElement(model, "pose").text = pose
        link = ET.SubElement(model, "link", {"name": "panel_link"}); visual = ET.SubElement(link, "visual", {"name": "panel_visual"}); geometry = ET.SubElement(visual, "geometry"); box = ET.SubElement(geometry, "box"); ET.SubElement(box, "size").text = size; material = ET.SubElement(visual, "material"); ET.SubElement(material, "diffuse").text = color

tree.write(destination, encoding="utf-8", xml_declaration=True)
