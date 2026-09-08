#!/usr/bin/env python3
"""Create an isolated runtime world by adding the current Kuruma to the pinned source scene."""
import sys
import xml.etree.ElementTree as ET

source, destination = sys.argv[1:]
tree = ET.parse(source)
world = tree.getroot().find('world')
if world is None:
    raise SystemExit('Pinned world has no world element')
world.set('name', 'default')
include = ET.SubElement(world, 'include')
ET.SubElement(include, 'uri').text = 'model://kuruma'
ET.SubElement(include, 'name').text = 'kuruma'
ET.SubElement(include, 'pose').text = '0 0 0.08 0 0 0'
tree.write(destination, encoding='utf-8', xml_declaration=True)
