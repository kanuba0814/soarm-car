#!/usr/bin/env bash
set -euo pipefail
# Static contract catches geometry/plugin/config regressions before the live Fortress seam.
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET

model = ET.parse('ros2_ws/src/soarm_simulation/models/kuruma/model.sdf').getroot().find("model[@name='kuruma']")
assert model is not None
plugin = model.find("plugin[@name='ignition::gazebo::systems::MecanumDrive']")
assert plugin is not None and plugin.attrib['filename'] == 'ignition-gazebo-mecanum-drive-system'
odometry = model.find("plugin[@name='ignition::gazebo::systems::OdometryPublisher']")
assert odometry is not None and odometry.findtext('odom_topic') == '/gazebo/odom'
assert odometry.findtext('odom_publish_frequency') == '50'
assert {node.tag: node.text for node in plugin}.items() >= {
    'front_left_joint': 'front_left_wheel_joint', 'front_right_joint': 'front_right_wheel_joint',
    'back_left_joint': 'rear_left_wheel_joint', 'back_right_joint': 'rear_right_wheel_joint',
    'wheel_separation': '0.4456', 'wheelbase': '0.3472', 'wheel_radius': '0.0763',
    'topic': '/safe_cmd_vel',
}.items()
joints = {joint.attrib['name']: joint for joint in model.findall('joint')}
assert set(name for name in joints if name.endswith('_wheel_joint')) == {
    'front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint'}
assert all(joints[name].findtext('axis/xyz') == '0 1 0' for name in joints if name.endswith('_wheel_joint'))
assert not any('steering' in name or name.startswith('arm_') for name in joints)
base = model.find("link[@name='base_link']")
frame_collisions = {
    collision.attrib['name']: (collision.findtext('pose'), collision.findtext('geometry/box/size'))
    for collision in base.findall('collision') if collision.attrib['name'].startswith('frame_')
}
assert frame_collisions == {
    'frame_front_collision': ('0 0.285 0 0 0 0', '0.600 0.030 0.030'),
    'frame_rear_collision': ('0 -0.285 0 0 0 0', '0.600 0.030 0.030'),
    'frame_left_collision': ('0.285 0 0 0 0 0', '0.030 0.540 0.030'),
    'frame_right_collision': ('-0.285 0 0 0 0 0', '0.030 0.540 0.030'),
}
wheels = [link for link in model.findall('link') if link.attrib['name'].endswith('_wheel')]
assert len(wheels) == 4
assert all(link.findtext('collision/surface/friction/ode/fdir1') for link in wheels)
assert {link.attrib['name'] for link in model.findall('link')} >= {'camera_link', 'gyro_link', 'livox_frame'}
xacro = Path('ros2_ws/src/soarm_description/urdf/soarm_car_layout.urdf.xacro').read_text()
for value in ('name="kuruma"', 'value="0.0763"', 'value="0.3472"', 'value="0.4456"', 'value="0.600"'):
    assert value in xacro
assert 'arm_mount' not in xacro and 'steering' not in xacro
adapter = Path('ros2_ws/src/soarm_simulation/soarm_simulation/sim_adapter.py').read_text()
assert "arm_enabled', False" in adapter and 'GoalResponse.REJECT' in adapter
assert "'/ground_truth/odom'" in adapter and "'/safe_cmd_vel'" in adapter
assert 'def on_joint_states' in adapter and 'cmd_vel timed out' in adapter
PY

docker compose run --rm ros2-dev bash -lc '
  source /opt/ros/humble/setup.bash
  source /tmp/soarm_ros2/install/setup.bash
  xacro /workspaces/soarm-car/ros2_ws/src/soarm_description/urdf/soarm_car_layout.urdf.xacro >/tmp/kuruma.urdf
  python3 - <<"PY"
import xml.etree.ElementTree as ET
root = ET.parse("/tmp/kuruma.urdf").getroot()
assert root.attrib["name"] == "kuruma"
joints = root.findall("joint")
assert len([j for j in joints if j.attrib["type"] == "continuous"]) == 4
assert not any("arm" in node.attrib.get("name", "") or "steering" in node.attrib.get("name", "") for node in root.iter())
PY
'
printf '%s\n' 'Kuruma static and xacro smoke passed.'

# Fortress 6 MecanumDrive moves the model but its bundled example does not emit
# odometry. The native OdometryPublisher companion reports the physical model state.
ros_domain_id="${ROS_DOMAIN_ID:-148}"
ign_partition="${IGN_PARTITION:-kuruma_sim_smoke_$$}"
container="$(docker compose run -d --rm -e ROS_DOMAIN_ID="$ros_domain_id" -e IGN_PARTITION="$ign_partition" ros2-dev bash -lc 'ros2 launch soarm_simulation simulation.launch.py')"
cleanup() { docker stop "$container" >/dev/null 2>&1 || true; }
trap cleanup EXIT
ros='source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && ros2'
for _ in {1..120}; do
    if docker exec "$container" bash -lc "test -f /tmp/soarm_ros2/install/setup.bash && output=\$($ros topic info /gazebo/odom) && [[ \$output == *'Publisher count: 1'* ]]"; then
        break
    fi
    sleep 1
done
docker exec "$container" bash -lc "test -f /tmp/soarm_ros2/install/setup.bash && output=\$($ros topic info /gazebo/odom) && [[ \$output == *'Publisher count: 1'* ]]"
docker exec "$container" bash -lc "source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && encoder=\$(ros2 topic info /odom) && truth=\$(ros2 topic info /ground_truth/odom) && [[ \$encoder == *'Publisher count: 1'* && \$truth == *'Publisher count: 1'* ]]"
docker exec -i "$container" bash -s <<'EOS'
source /opt/ros/humble/setup.bash
source /tmp/soarm_ros2/install/setup.bash
snapshot() { timeout 8 ros2 topic echo /gazebo/odom --once; }
for axis in forward lateral rotate; do
    before="$(snapshot)"
    case "$axis" in
        forward) command='{linear: {x: 0.2, y: 0.0}, angular: {z: 0.0}}' ;;
        lateral) command='{linear: {x: 0.0, y: 0.2}, angular: {z: 0.0}}' ;;
        rotate) command='{linear: {x: 0.0, y: 0.0}, angular: {z: 0.4}}' ;;
    esac
    ros2 topic pub -r 20 --times 80 /cmd_vel geometry_msgs/msg/Twist "$command" >/dev/null 2>&1 & publisher=$!
    sleep 2
    after="$(snapshot)"
    wait "$publisher"
    ros2 topic pub -r 20 --times 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}' >/dev/null
    printf '%s\n---BEFORE---\n%s\n---AFTER---\n%s\n' "$axis" "$before" "$after" > "/tmp/kuruma-${axis}.yaml"
done
encoder_before="$(timeout 8 ros2 topic echo /odom --once)"
truth_before="$(timeout 8 ros2 topic echo /ground_truth/odom --once)"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}' >/dev/null
sleep 1
encoder_moving="$(timeout 8 ros2 topic echo /odom --once)"
truth_moving="$(timeout 8 ros2 topic echo /ground_truth/odom --once)"
sleep 2
truth_stopped="$(timeout 8 ros2 topic echo /ground_truth/odom --once)"
printf '%s\n---MOVING---\n%s\n' "$encoder_before" "$encoder_moving" > /tmp/kuruma-encoder.yaml
printf '%s\n---MOVING---\n%s\n---STOPPED---\n%s\n' "$truth_before" "$truth_moving" "$truth_stopped" > /tmp/kuruma-timeout.yaml
EOS
for axis in forward lateral rotate encoder timeout; do docker cp "$container:/tmp/kuruma-${axis}.yaml" "/tmp/kuruma-${axis}.yaml"; done
python3 - <<'PY'
import math
import re

pairs = {}
for name in ('forward', 'lateral', 'rotate'):
    before, after = open(f'/tmp/kuruma-{name}.yaml').read().split('---AFTER---', 1)
    def pose(text):
        x, y = map(float, re.search(r'position:\n      x: ([^\n]+)\n      y: ([^\n]+)', text).groups())
        z, w = map(float, re.search(r'orientation:\n      x: [^\n]+\n      y: [^\n]+\n      z: ([^\n]+)\n      w: ([^\n]+)', text).groups())
        return x, y, 2 * math.atan2(z, w)
    x, y, yaw = pose(before)
    next_x, next_y, next_yaw = pose(after)
    dx, dy = next_x - x, next_y - y
    pairs[name] = (math.cos(yaw) * dx + math.sin(yaw) * dy,
                   -math.sin(yaw) * dx + math.cos(yaw) * dy,
                   next_yaw - yaw)
forward, lateral, rotate = pairs['forward'], pairs['lateral'], pairs['rotate']
assert abs(forward[0]) > 0.06 and abs(forward[0]) > abs(forward[1]) * 1.5, pairs
assert abs(lateral[1]) > 0.06 and abs(lateral[1]) > abs(lateral[0]) * 1.5, pairs
assert abs(rotate[2]) > 0.15 and math.hypot(rotate[0], rotate[1]) < 0.08, pairs

def point(text):
    return tuple(map(float, re.search(r'position:\n      x: ([^\n]+)\n      y: ([^\n]+)', text).groups()))
def distance(before, after):
    x, y = point(before); next_x, next_y = point(after)
    return math.hypot(next_x - x, next_y - y)
encoder_before, encoder_moving = open('/tmp/kuruma-encoder.yaml').read().split('---MOVING---', 1)
truth_before, remainder = open('/tmp/kuruma-timeout.yaml').read().split('---MOVING---', 1)
truth_moving, truth_stopped = remainder.split('---STOPPED---', 1)
encoder_distance = distance(encoder_before, encoder_moving)
truth_distance = distance(truth_before, truth_moving)
post_timeout_drift = distance(truth_moving, truth_stopped)
assert encoder_distance > 0.01 and truth_distance > 0.01, (encoder_distance, truth_distance)
assert post_timeout_drift < 0.01, post_timeout_drift
print('Kuruma physical odometry passed:', pairs)
print('Encoder/ground-truth split and command timeout passed:', encoder_distance, truth_distance, post_timeout_drift)
PY
