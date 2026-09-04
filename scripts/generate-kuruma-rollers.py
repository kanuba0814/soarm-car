#!/usr/bin/env python3
"""Deterministically generate the explicit-roller prototype from the supplied STEP."""
from __future__ import annotations
import json
import math
from pathlib import Path
import numpy as np
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader

ROOT = Path(__file__).resolve().parents[1]
STEP = ROOT / "artifacts/kuruma.STEP"
MODEL = ROOT / "ros2_ws/src/soarm_simulation/models/kuruma_rollers"
MEASURED = {
    "source": "artifacts/kuruma.STEP read with OCP 7.9.3.1",
    "rollers_per_wheel": 16, "spacing_deg": 22.5,
    "center_ring_radius_m": 0.0668865, "axis_axle_component": 0.705615,
    "axis_tangent_component": 0.708572, "axis_radial_residual": 0.005724,
    "roller_length_m": 0.038, "roller_max_radius_m": 0.0092268,
    "roller_volume_m3": 4.27366e-6, "wheel_envelope_radius_m": 0.0763,
    "wheel_width_m": 0.0481,
}

def check_step():
    reader = STEPControl_Reader()
    if reader.ReadFile(str(STEP)) != IFSelect_RetDone or reader.TransferRoots() < 1 or reader.NbShapes() < 1:
        raise RuntimeError("STEP is not a transferable OCP shape")

def mesh():
    # STEP-measured capsule envelope, centered at origin and with its spin axis +Z.
    r, length, rings, sides = MEASURED["roller_max_radius_m"], MEASURED["roller_length_m"], 8, 20
    half = length / 2 - r
    vertices, faces = [], []
    for ring in range(2 * rings + 1):
        phi = -math.pi / 2 + math.pi * ring / (2 * rings)
        z = r * math.sin(phi) + (-half if ring < rings else half)
        for side in range(sides):
            a = 2 * math.pi * side / sides
            vertices.append((r * math.cos(phi) * math.cos(a), r * math.cos(phi) * math.sin(a), z))
    for ring in range(2 * rings):
        for side in range(sides):
            n = (side + 1) % sides; a = ring * sides + side + 1; b = ring * sides + n + 1
            faces.extend(((a, b, b + sides), (a, b + sides, a + sides)))
    vertex_lines = "".join(f"v {x:.9f} {y:.9f} {z:.9f}\n" for x, y, z in vertices)
    # DART's ODE mesh collision requires a normal for every vertex.
    normal_lines = "".join(f"vn {x:.9f} {y:.9f} 0.000000000\n" for x, y, _ in vertices)
    return "# CAD-measured roller envelope; centered, +Z joint axis\n" + vertex_lines + normal_lines + "".join(f"f {a}//{a} {b}//{b} {c}//{c}\n" for a, b, c in faces)

def pose_for_axis(axis):
    return 0.0, math.acos(max(-1, min(1, axis[2]))), math.atan2(axis[1], axis[0])

def roller(wheel, hand, index):
    a = 2 * math.pi * index / 16
    radial = np.array([math.cos(a), 0., math.sin(a)])
    tangent = np.array([-math.sin(a), 0., math.cos(a)])
    axis = MEASURED["axis_axle_component"] * np.array([0., 1., 0.]) + hand * MEASURED["axis_tangent_component"] * tangent
    axis /= np.linalg.norm(axis)
    roll, pitch, yaw = pose_for_axis(axis)
    center = MEASURED["center_ring_radius_m"] * radial
    name = f"{wheel}_roller_{index:02d}"
    return f'''<link name="{name}"><pose relative_to="{wheel}">{center[0]:.9f} 0 {center[2]:.9f} {roll:.9f} {pitch:.9f} {yaw:.9f}</pose><inertial><mass>0.035</mass><inertia><ixx>0.000006</ixx><iyy>0.000006</iyy><izz>0.000002</izz></inertia></inertial><visual name="roller"><geometry><mesh><uri>model://kuruma_rollers/meshes/roller.obj</uri></mesh></geometry></visual><collision name="roller_collision"><geometry><capsule><radius>0.0092268</radius><length>0.0195464</length></capsule></geometry><surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision></link><joint name="{name}_joint" type="revolute"><parent>{wheel}</parent><child>{name}</child><axis><xyz expressed_in="{name}">0 0 1</xyz><limit><lower>-1.79769e+308</lower><upper>1.79769e+308</upper><effort>5</effort><velocity>100</velocity></limit><dynamics><damping>0.00002</damping><friction>0.00001</friction></dynamics></axis></joint>'''

def sdf():
    wheels = (("front_left_wheel", .1736, .2228, -1), ("front_right_wheel", .1736, -.2228, 1), ("rear_left_wheel", -.1736, .2228, 1), ("rear_right_wheel", -.1736, -.2228, -1))
    items = []
    for name,x,y,hand in wheels:
        items.append(f'''<link name="{name}"><pose relative_to="base_link">{x} {y} 0 0 0 0</pose><inertial><mass>0.40</mass><inertia><ixx>0.001</ixx><iyy>0.0005</iyy><izz>0.001</izz></inertia></inertial><visual name="hub"><pose>0 0 0 1.570796 0 0</pose><geometry><cylinder><radius>0.048</radius><length>0.040</length></cylinder></geometry></visual><collision name="hub_collision"><pose>0 0 0 1.570796 0 0</pose><geometry><cylinder><radius>0.048</radius><length>0.040</length></cylinder></geometry><surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision></link><joint name="{name}_joint" type="revolute"><parent>base_link</parent><child>{name}</child><axis><xyz>0 1 0</xyz><limit><lower>-1.79769e+308</lower><upper>1.79769e+308</upper><effort>20</effort><velocity>30</velocity></limit><dynamics><damping>0.001</damping></dynamics></axis></joint>''')
        items.extend(roller(name, hand, i) for i in range(16))
    base = '''<link name="base_link"><pose relative_to="__model__">0 0 0.0785 0 0 0</pose><inertial><mass>12</mass><inertia><ixx>0.40</ixx><iyy>0.40</iyy><izz>0.70</izz></inertia></inertial><collision name="frame_front_collision"><pose>0 0.285 0 0 0 0</pose><geometry><box><size>0.600 0.030 0.030</size></box></geometry></collision><collision name="frame_rear_collision"><pose>0 -0.285 0 0 0 0</pose><geometry><box><size>0.600 0.030 0.030</size></box></geometry></collision><collision name="frame_left_collision"><pose>0.285 0 0 0 0 0</pose><geometry><box><size>0.030 0.540 0.030</size></box></geometry></collision><collision name="frame_right_collision"><pose>-0.285 0 0 0 0 0</pose><geometry><box><size>0.030 0.540 0.030</size></box></geometry></collision><collision name="electronics_collision"><geometry><box><size>0.360 0.360 0.060</size></box></geometry></collision></link>'''
    plugins = '''<plugin filename="ignition-gazebo-mecanum-drive-system" name="ignition::gazebo::systems::MecanumDrive"><front_left_joint>front_left_wheel_joint</front_left_joint><front_right_joint>front_right_wheel_joint</front_right_joint><back_left_joint>rear_left_wheel_joint</back_left_joint><back_right_joint>rear_right_wheel_joint</back_right_joint><wheel_separation>0.4456</wheel_separation><wheelbase>0.3472</wheelbase><wheel_radius>0.0763</wheel_radius><topic>/safe_cmd_vel</topic><min_acceleration>-2.0</min_acceleration><max_acceleration>2.0</max_acceleration></plugin><plugin filename="ignition-gazebo-odometry-publisher-system" name="ignition::gazebo::systems::OdometryPublisher"><odom_frame>kuruma_rollers/odom</odom_frame><robot_base_frame>kuruma_rollers/base_link</robot_base_frame><odom_publish_frequency>50</odom_publish_frequency><odom_topic>/gazebo/odom</odom_topic></plugin><plugin filename="ignition-gazebo-joint-state-publisher-system" name="ignition::gazebo::systems::JointStatePublisher"><topic>/joint_states</topic></plugin>'''
    return '<?xml version="1.0"?>\n<sdf version="1.9"><model name="kuruma_rollers"><!-- CAD envelope; contact/mass/damping are provisional, not motor fidelity. -->' + base + ''.join(items) + plugins + '</model></sdf>\n'

def main():
    check_step(); MODEL.joinpath("meshes").mkdir(parents=True, exist_ok=True)
    MODEL.joinpath("meshes/roller.obj").write_text(mesh())
    MODEL.joinpath("model.sdf").write_text(sdf())
    MODEL.joinpath("measurements.json").write_text(json.dumps(MEASURED, indent=2, sort_keys=True) + "\n")
if __name__ == "__main__": main()
