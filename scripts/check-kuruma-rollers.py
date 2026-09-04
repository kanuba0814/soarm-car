#!/usr/bin/env python3
"""Static contract for the generated explicit-roller prototype."""
import json, math
from pathlib import Path
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
model_path = root / 'ros2_ws/src/soarm_simulation/models/kuruma_rollers/model.sdf'
measurements = json.loads((model_path.parent / 'measurements.json').read_text())
model = ET.parse(model_path).getroot().find("model[@name='kuruma_rollers']")
assert model is not None
joints = {j.attrib['name']: j for j in model.findall('joint')}
hubs = [n for n in joints if n.endswith('_wheel_joint')]
rollers = [n for n in joints if '_roller_' in n and n.endswith('_joint')]
assert len(hubs) == 4, hubs
assert len(rollers) == 64, len(rollers)
assert not model.findall(".//fdir1")
assert not any(c.findtext('surface/friction/ode/mu2') == '0.0' for c in model.findall('.//collision'))
assert measurements['rollers_per_wheel'] == 16 and measurements['spacing_deg'] == 22.5
for wheel, expected_hand in {'front_left_wheel': -1, 'front_right_wheel': 1, 'rear_left_wheel': 1, 'rear_right_wheel': -1}.items():
    links = [l for l in model.findall('link') if l.attrib['name'].startswith(wheel + '_roller_')]
    assert len(links) == 16, wheel
    angles = []
    for link in links:
        x, _, z, roll, pitch, yaw = map(float, link.findtext('pose').split())
        assert abs(math.hypot(x, z) - measurements['center_ring_radius_m']) < 1e-8
        capsule = link.find('collision/geometry/capsule')
        assert capsule is not None
        assert abs(float(capsule.findtext('radius')) - measurements['roller_max_radius_m']) < 1e-9
        assert abs(float(capsule.findtext('length')) + 2 * float(capsule.findtext('radius')) - measurements['roller_length_m']) < 1e-9
        # Generator maps link +Z to this axis; its tangent component carries the hand.
        tangent_component = (-math.sin(math.atan2(z, x))) * (math.cos(yaw) * math.sin(pitch)) + math.cos(math.atan2(z, x)) * math.cos(pitch)
        assert tangent_component * expected_hand > 0.70, (link.attrib['name'], tangent_component)
        angles.append(math.atan2(z, x))
    angles.sort()
    steps = [((angles[(i + 1) % 16] - angles[i]) % (2 * math.pi)) for i in range(16)]
    assert all(abs(step - math.radians(22.5)) < 1e-8 for step in steps), steps
assert (model_path.parent / 'meshes/roller.obj').read_text().startswith('# CAD-measured')
print('kuruma_rollers static contract passed: 4 hubs, 64 passive rollers, CAD ring/spacing/hand verified')
