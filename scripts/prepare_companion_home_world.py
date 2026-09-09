#!/usr/bin/env python3
"""Create an isolated runtime world by adding the current Kuruma to the pinned source scene."""
import sys
import xml.etree.ElementTree as ET

source, destination, *options = sys.argv[1:]
if options and options != ["--metric-calibration-panel"]:
    raise SystemExit("usage: prepare_companion_home_world.py SOURCE DESTINATION [--metric-calibration-panel]")
tree = ET.parse(source)
world = tree.getroot().find('world')
if world is None:
    raise SystemExit('Pinned world has no world element')
world.set('name', 'default')
include = ET.SubElement(world, 'include')
ET.SubElement(include, 'uri').text = 'model://kuruma'
ET.SubElement(include, 'name').text = 'kuruma'
ET.SubElement(include, 'pose').text = '0 0 0.08 0 0 0'
if options:
    # Runtime-only known plane for sensor convention calibration; it is not a home-study target.
    panel = ET.SubElement(world, 'model', {'name': 'metric_calibration_panel'})
    ET.SubElement(panel, 'static').text = 'true'
    ET.SubElement(panel, 'pose').text = '-5.0 0.9 0.5 0 0 0'
    link = ET.SubElement(panel, 'link', {'name': 'panel_link'})
    visual = ET.SubElement(link, 'visual', {'name': 'panel_visual'})
    geometry = ET.SubElement(visual, 'geometry')
    box = ET.SubElement(geometry, 'box')
    ET.SubElement(box, 'size').text = '0.02 2.0 2.0'
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'diffuse').text = '0.0 1.0 0.0 1.0'
    collision = ET.SubElement(link, 'collision', {'name': 'panel_collision'})
    collision_geometry = ET.SubElement(collision, 'geometry')
    collision_box = ET.SubElement(collision_geometry, 'box')
    ET.SubElement(collision_box, 'size').text = '0.02 2.0 2.0'
    # Orthogonal blue panel: visible from the fixed 90-degree diagnostic heading.
    panel_y = ET.SubElement(world, 'model', {'name': 'metric_calibration_panel_y'})
    ET.SubElement(panel_y, 'static').text = 'true'
    ET.SubElement(panel_y, 'pose').text = '-6.9 2.8 0.5 0 0 0'
    link_y = ET.SubElement(panel_y, 'link', {'name': 'panel_link'})
    visual_y = ET.SubElement(link_y, 'visual', {'name': 'panel_visual'})
    geometry_y = ET.SubElement(visual_y, 'geometry')
    box_y = ET.SubElement(geometry_y, 'box')
    ET.SubElement(box_y, 'size').text = '2.0 0.02 2.0'
    material_y = ET.SubElement(visual_y, 'material')
    ET.SubElement(material_y, 'diffuse').text = '0.0 0.0 1.0 1.0'
    collision_y = ET.SubElement(link_y, 'collision', {'name': 'panel_collision'})
    collision_geometry_y = ET.SubElement(collision_y, 'geometry')
    collision_box_y = ET.SubElement(collision_geometry_y, 'box')
    ET.SubElement(collision_box_y, 'size').text = '2.0 0.02 2.0'
tree.write(destination, encoding='utf-8', xml_declaration=True)
