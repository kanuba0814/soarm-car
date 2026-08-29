#!/usr/bin/env bash
set -euo pipefail
# Full headless Fortress acceptance seam: native Ackermann entity, sensors, TF and actions.
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
model_path = Path('ros2_ws/src/soarm_simulation/models/senior_akm/model.sdf')
root = ET.parse(model_path).getroot()
model = root.find("./model[@name='senior_akm']")
plugin = model.find("plugin[@name='ignition::gazebo::systems::AckermannSteering']")
assert [node.text for node in plugin.findall('left_joint')] == ['left_rear_wheel_joint']
assert [node.text for node in plugin.findall('right_joint')] == ['right_rear_wheel_joint']
links = {link.attrib['name']: link for link in model.findall('link')}
joints = {joint.attrib['name']: joint for joint in model.findall('joint')}
def pose(link_name):
    node = links[link_name].find('pose')
    assert node is not None and 'relative_to' in node.attrib, link_name
    assert node.text is not None, link_name
    return node.attrib['relative_to'], [float(value) for value in node.text.split()]
def joint_axis(joint_name):
    return [float(value) for value in joints[joint_name].find('axis/xyz').text.split()]
def assert_pose(link_name, relative_to, values):
    actual_relative_to, actual_values = pose(link_name)
    assert actual_relative_to == relative_to, (link_name, actual_relative_to)
    assert all(abs(a - b) < 1e-9 for a, b in zip(actual_values, values)), (link_name, actual_values)
# Every vehicle link is explicitly located in its parent frame; every joint pose is its child origin.
for name, link in links.items():
    node = link.find('pose')
    assert node is not None and 'relative_to' in node.attrib, name
for name, joint in joints.items():
    node = joint.find('pose')
    assert node is not None and node.attrib.get('relative_to') == joint.findtext('child'), name
    assert node.text is not None, name
    assert [float(value) for value in node.text.split()] == [0.0] * 6, name
assert pose('base_link')[0] == '__model__'
assert pose('camera_link')[0] == 'second_deck'
assert pose('livox_frame')[0] == 'third_deck'
assert pose('arm_mount')[0] == 'third_deck'
assert '0.400 0.260 0.040' in [node.text for node in links['base_link'].iter('size')]
assert_pose('third_deck', 'base_link', [0, 0, 0.137, 0, 0, 0])
assert '0.445 0.320 0.005' in [node.text for node in links['third_deck'].iter('size')]
assert_pose('left_front_wheel', 'left_front_steering_link', [0, 0, 0, 0, 0, 0])
assert_pose('right_front_wheel', 'right_front_steering_link', [0, 0, 0, 0, 0, 0])
assert_pose('left_rear_wheel', 'base_link', [-0.1725, 0.155, 0, 0, 0, 0])
assert_pose('right_rear_wheel', 'base_link', [-0.1725, -0.155, 0, 0, 0, 0])
for wheel in ('left_front_wheel', 'right_front_wheel', 'left_rear_wheel', 'right_rear_wheel'):
    tire_visual = next(node for node in links[wheel].findall('visual') if node.attrib.get('name') == 'tire_visual')
    tire_collision = next(node for node in links[wheel].findall('collision') if node.attrib.get('name') == 'tire')
    assert tire_visual.find('pose').text.split()[3:] == ['-1.570796', '0', '0']
    assert tire_collision.find('pose').text.split()[3:] == ['-1.570796', '0', '0']
    assert joint_axis(f'{wheel}_joint') == [0.0, 1.0, 0.0]
for steering in ('left_front_steering_joint', 'right_front_steering_joint'):
    assert joint_axis(steering) == [0.0, 0.0, 1.0]
assert joints['left_front_wheel_joint'].findtext('parent') == 'left_front_steering_link'
assert joints['right_front_wheel_joint'].findtext('parent') == 'right_front_steering_link'
assert len([name for name in links if name.startswith('pillar_')]) == 8
for name in ('pillar_fl_upper', 'pillar_fr_upper', 'pillar_rl_upper', 'pillar_rr_upper'):
    assert abs(pose(name)[1][2] - 0.0185) < 1e-9, (name, pose(name))
PY
container="$(docker compose run -d --rm ros2-dev bash -lc 'ros2 launch soarm_simulation simulation.launch.py')"
cleanup() { docker stop "$container" >/dev/null 2>&1 || true; }
trap cleanup EXIT
ros_command='source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && ros2'
for _ in {1..45}; do
    if docker exec "$container" bash -lc "test -f /tmp/soarm_ros2/install/setup.bash" && docker exec "$container" bash -lc "$ros_command action list | grep -qx /go_to_place_and_grasp"; then break; fi
    sleep 1
done
# Action discovery can precede sensor bridges and native controllers becoming ready.
sleep 3
inside() { docker exec "$container" bash -lc "$ros_command $*"; }
inside_timeout() { local seconds=$1 command; shift; printf -v command '%q ' "$@"; docker exec "$container" bash -lc "source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && timeout $seconds $command"; }
contains() { local output; output="$(inside "$1")"; grep -q -- "$2" <<<"$output"; }
has_publisher() { local output; output="$(inside "topic info $1")"; grep -Eq 'Publisher count: [1-9][0-9]*' <<<"$output"; }
docker exec "$container" bash -lc "test -f /tmp/soarm_ros2/install/setup.bash"
contains 'action list' '^/go_to_place_and_grasp$'
for topic in /odom /gazebo/odom /imu/data_raw /camera/image /camera/camera_info /mid360/points /joint_states /PowerVoltage; do
    has_publisher "$topic"
done
for action in /go_to_place_and_grasp /arm_controller/follow_joint_trajectory /gripper_controller/follow_joint_trajectory; do
    contains 'action list' "^$action$"
done
# Physical Fortress motion: the native Ackermann model must move and turn; lateral Twist is ignored.
# timeout owns the publisher lifecycle inside the container, avoiding a stale command publisher.
inside_timeout 4 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.9}, angular: {z: 0.25}}" >/tmp/soarm-drive.log 2>&1 & publisher=$!
sleep 3
inside "topic echo /gazebo/odom --once" >/tmp/soarm-gazebo-drive.yaml
inside "topic echo /odom --once" >/tmp/soarm-odom-drive.yaml
wait "$publisher" || true
# Keep the zero command alive long enough for the ROS->Gazebo bridge.
inside_timeout 2 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
sleep 1
inside_timeout 10 ros2 topic echo /gazebo/odom --once >/tmp/soarm-gazebo-stop.yaml
printf '%s\n' 'Checking physical Ackermann motion...'
python3 - <<'PY'
import re
for name in ('/tmp/soarm-gazebo-drive.yaml', '/tmp/soarm-odom-drive.yaml'):
    text = open(name).read()
    match = re.search(r'position:\n      x: ([^\n]+)\n      y: ([^\n]+)', text)
    assert match and abs(float(match.group(1))) > 0.1, (name, match)
assert 'frame_id: senior_akm/odom' in open('/tmp/soarm-gazebo-drive.yaml').read()
assert 'frame_id: odom_combined' in open('/tmp/soarm-odom-drive.yaml').read()
drive = open('/tmp/soarm-gazebo-drive.yaml').read()
match = re.search(r'twist:\n  twist:\n    linear:\n      x: ([^\n]+)\n      y: ([^\n]+)', drive)
assert match and abs(float(match.group(1)) - 0.2) < 0.03 and abs(float(match.group(2))) < 1e-9, drive
stop = open('/tmp/soarm-gazebo-stop.yaml').read()
match = re.search(r'twist:\n  twist:\n    linear:\n      x: ([^\n]+)\n      y: ([^\n]+)', stop)
assert match and abs(float(match.group(1))) < 1e-5 and abs(float(match.group(2))) < 1e-9, stop
PY
printf '%s\n' 'Checking vehicle-mounted sensors...'
# Consumer message/frame checks. Gazebo sensors use sensor-data (best-effort) QoS.
inside_timeout 10 ros2 topic echo /imu/data_raw --once --qos-reliability best_effort >/tmp/soarm-imu.yaml
inside_timeout 10 ros2 topic echo /camera/image --once --qos-reliability best_effort >/tmp/soarm-image.yaml
inside_timeout 10 ros2 topic echo /camera/camera_info --once --qos-reliability best_effort >/tmp/soarm-camera-info.yaml
inside_timeout 10 ros2 topic echo /mid360/points --once --qos-reliability best_effort >/tmp/soarm-points.yaml
grep -q 'frame_id: gyro_link' /tmp/soarm-imu.yaml
grep -q 'width: 640' /tmp/soarm-image.yaml
grep -q 'height: 480' /tmp/soarm-camera-info.yaml
grep -q 'frame_id: livox_frame' /tmp/soarm-points.yaml
printf '%s\n' 'Checking physical arm and controller seams...'
# The action adapter must drive native Fortress joints, and /joint_states must be the bridged state.
inside_timeout 10 ros2 topic echo /joint_states --once >/tmp/soarm-joints-before.yaml
arm_result="$(inside "action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{trajectory: {joint_names: [arm_shoulder_pan, arm_shoulder_lift, arm_elbow_flex, arm_wrist_flex, arm_wrist_roll], points: [{positions: [0.1, -0.1, 0.08, -0.08, 0.1], time_from_start: {sec: 2}}]}}\"")"
if ! grep -q 'Goal finished with status: SUCCEEDED' <<<"$arm_result"; then
    printf '%s\n' "$arm_result" >&2
    exit 1
fi
gripper_result="$(inside "action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{trajectory: {joint_names: [arm_gripper_opening], points: [{positions: [0.4], time_from_start: {sec: 1}}]}}\"")"
if ! grep -q 'Goal finished with status: SUCCEEDED' <<<"$gripper_result"; then
    printf '%s\n' "$gripper_result" >&2
    exit 1
fi
inside_timeout 10 ros2 topic echo /joint_states --once >/tmp/soarm-joints-after.yaml
python3 - <<'PY'
import re

def position(text, name):
    names = re.search(r'name:\n((?:\s*- .*\n)+)', text)
    values = re.search(r'position:\n((?:\s*- .*\n)+)', text)
    assert names and values, text
    joint_names = [line.split('- ', 1)[1].strip() for line in names.group(1).splitlines()]
    joint_values = [float(line.split('- ', 1)[1]) for line in values.group(1).splitlines()]
    return dict(zip(joint_names, joint_values))[name]

before = open('/tmp/soarm-joints-before.yaml').read()
after = open('/tmp/soarm-joints-after.yaml').read()
targets = {
    'arm_shoulder_pan': 0.1,
    'arm_shoulder_lift': -0.1,
    'arm_elbow_flex': 0.08,
    'arm_wrist_flex': -0.08,
    'arm_wrist_roll': 0.1,
}
for joint, target in targets.items():
    assert abs(position(after, joint) - target) < 0.05, (joint, after)
# The gripper Action seam is exercised above, but contact/position convergence is not
# accepted as physical evidence for the coarse single-servo gripper model.
assert abs(position(after, 'arm_shoulder_lift') - position(before, 'arm_shoulder_lift')) > 0.05, (before, after)
PY
printf '%s\n' 'Checking mission success semantics, rejection, conflicts, and cancellation...'
# Valid action, rejection, and cancellation all use the live action server.
success="$(inside "action send_goal /go_to_place_and_grasp soarm_interfaces/action/GoToPlaceAndGrasp \"{request_id: demo-1, place_id: demo_table, object_id: demo_cube}\" --feedback")"
grep -q 'success: true' <<<"$success"
grep -q 'final_stage: complete' <<<"$success"
grep -q 'object grasp not verified' <<<"$success"
invalid="$(inside "action send_goal /go_to_place_and_grasp soarm_interfaces/action/GoToPlaceAndGrasp \"{request_id: invalid-1, place_id: nowhere, object_id: demo_cube}\"")"
grep -q 'Goal was rejected.' <<<"$invalid"
docker exec "$container" bash -lc "source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && timeout 35 python3 /workspaces/soarm-car/scripts/check-sim-cancel.py"
sleep 1
inside_timeout 10 ros2 topic echo /gazebo/odom --once >/tmp/soarm-cancel-stop.yaml
python3 - <<'PY'
import re
text = open('/tmp/soarm-cancel-stop.yaml').read()
match = re.search(r'twist:\n  twist:\n    linear:\n      x: ([^\n]+)', text)
assert match and abs(float(match.group(1))) < 1e-5, text
PY
printf '%s\n' 'Headless Fortress simulation smoke passed.'
