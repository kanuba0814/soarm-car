#!/usr/bin/env bash
set -euo pipefail

python3 - <<'PY'
import xml.etree.ElementTree as ET

model = ET.parse('ros2_ws/src/soarm_simulation/models/kuruma_ros2_control/model.sdf').getroot().find('model')
axes = {
    'front_left': ('0.70710678 0.70710678 0', '0.70710678 -0.70710678 0'),
    'front_right': ('-0.70710678 0.70710678 0', '0.70710678 0.70710678 0'),
    'rear_left': ('-0.70710678 0.70710678 0', '0.70710678 0.70710678 0'),
    'rear_right': ('0.70710678 0.70710678 0', '0.70710678 -0.70710678 0'),
}
for wheel, (axis, friction_direction) in axes.items():
    link = model.find(f"link[@name='{wheel}_wheel']")
    assert link.find('visual/geometry/sphere/radius').text == '0.0763'
    assert link.find('collision/geometry/sphere/radius').text == '0.0763'
    friction = link.find('collision/surface/friction/ode')
    assert friction.find('mu2').text == '0.0'
    assert friction.find('fdir1').text == friction_direction
    assert friction.find('fdir1').get('{http://ignitionrobotics.org/schema}expressed_in') == 'base_link'
    assert link.find('visual/geometry/cylinder') is None
    assert link.find('collision/geometry/cylinder') is None
    joint_axis = model.find(f"joint[@name='{wheel}_wheel_joint']/axis/xyz")
    assert joint_axis.text == axis
    assert joint_axis.get('expressed_in') == 'base_link'
PY

ros_domain_id="${ROS_DOMAIN_ID:-149}"
partition="kuruma_ros2_control_smoke_$$"
container="$(docker compose run -d --rm -e ROS_DOMAIN_ID="$ros_domain_id" -e IGN_PARTITION="$partition" ros2-dev bash -lc 'ros2 launch soarm_simulation kuruma_ros2_control.launch.py')"
on_exit() {
    status=$?
    trap - EXIT
    if (( status != 0 )); then
        docker logs "$container" > /tmp/kuruma-ros2-control-smoke.log 2>&1 || true
        tail -120 /tmp/kuruma-ros2-control-smoke.log >&2 || true
        echo 'Kuruma ros2_control smoke failed; full log: /tmp/kuruma-ros2-control-smoke.log' >&2
    fi
    docker stop "$container" >/dev/null 2>&1 || true
    exit "$status"
}
trap on_exit EXIT
ros='source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && ros2'

for _ in {1..120}; do
    if docker exec "$container" test -f /tmp/soarm_ros2/install/setup.bash; then
        break
    fi
    sleep 1
done
docker exec "$container" test -f /tmp/soarm_ros2/install/setup.bash

controllers=''
for _ in {1..120}; do
    controllers="$(docker exec "$container" bash -lc "$ros control list_controllers 2>/dev/null" || true)"
    if grep -q 'mecanum_drive_controller.*active' <<<"$controllers"; then
        break
    fi
    sleep 1
done
grep -q 'joint_state_broadcaster.*active' <<<"$controllers"
grep -q 'mecanum_drive_controller.*active' <<<"$controllers"

interfaces="$(docker exec "$container" bash -lc "$ros control list_hardware_interfaces")"
for joint in front_left front_right rear_left rear_right; do
    grep -q "${joint}_wheel_joint/velocity \[available\] \[claimed\]" <<<"$interfaces"
    grep -q "${joint}_wheel_joint/position" <<<"$interfaces"
    grep -q "${joint}_wheel_joint/velocity$" <<<"$interfaces"
done

wheel_info="$(docker exec "$container" bash -lc "$ros topic info /wheel/odom_raw")"
truth_info="$(docker exec "$container" bash -lc "$ros topic info /ground_truth/odom")"
grep -q 'Publisher count: 1' <<<"$wheel_info"
grep -q 'Publisher count: 1' <<<"$truth_info"
topics="$(docker exec "$container" bash -lc "$ros topic list")"
! grep -qx '/odom' <<<"$topics"
docker exec "$container" bash -lc "timeout 3 bash -lc 'source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && ros2 run tf2_ros tf2_echo wheel_odom base_link' >/tmp/kuruma-wheel-tf.txt 2>&1 || true"
wheel_tf="$(docker exec "$container" cat /tmp/kuruma-wheel-tf.txt)"
if grep -q 'Translation:' <<<"$wheel_tf"; then
    echo 'unexpected wheel_odom -> base_link TF owner' >&2
    exit 1
fi
docker exec "$container" bash -lc "timeout 3 bash -lc 'source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && ros2 run tf2_ros tf2_echo map base_link' >/tmp/kuruma-map-tf.txt 2>&1 || true"
map_tf="$(docker exec "$container" cat /tmp/kuruma-map-tf.txt)"
if grep -q 'Translation:' <<<"$map_tf"; then
    echo 'unexpected map -> base_link TF owner in plant-only profile' >&2
    exit 1
fi

docker exec -i "$container" bash -lc "$ros run rclpy --help >/dev/null 2>&1 || true; $ros pkg prefix rclpy >/dev/null; python3 -" <<'PY'
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

WHEELS = (
    'front_left_wheel_joint', 'front_right_wheel_joint',
    'rear_left_wheel_joint', 'rear_right_wheel_joint',
)


def yaw(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def pose(message):
    position = message.pose.pose.position
    return position.x, position.y, yaw(message.pose.pose.orientation)


def local_delta(before, after):
    x0, y0, heading = before
    x1, y1, next_heading = after
    dx, dy = x1 - x0, y1 - y0
    return (
        math.cos(heading) * dx + math.sin(heading) * dy,
        -math.sin(heading) * dx + math.cos(heading) * dy,
        math.atan2(math.sin(next_heading - heading), math.cos(next_heading - heading)),
    )


def distance(first, second):
    return math.hypot(second[0] - first[0], second[1] - first[1])


rclpy.init()
node = rclpy.create_node('kuruma_ros2_control_acceptance')
latest_truth = None
latest_wheel_odom = None
latest_joints = None
truth_samples = []


def on_truth(message):
    global latest_truth
    latest_truth = message
    truth_samples.append((time.monotonic(), message.header.stamp.sec + message.header.stamp.nanosec / 1e9, pose(message)))


def on_wheel_odom(message):
    global latest_wheel_odom
    latest_wheel_odom = message


def on_joints(message):
    global latest_joints
    if all(name in message.name for name in WHEELS):
        latest_joints = message


node.create_subscription(Odometry, '/ground_truth/odom', on_truth, 50)
node.create_subscription(Odometry, '/wheel/odom_raw', on_wheel_odom, 50)
node.create_subscription(JointState, '/joint_states', on_joints, 50)
publisher = node.create_publisher(Twist, '/cmd_vel', 10)


def spin(duration=0.1):
    end = time.monotonic() + duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=min(0.05, end - time.monotonic()))


def wait_ready():
    end = time.monotonic() + 20.0
    while time.monotonic() < end and (latest_truth is None or latest_wheel_odom is None or latest_joints is None):
        spin(0.1)
    assert latest_truth is not None, 'no /ground_truth/odom'
    assert latest_wheel_odom is not None, 'no /wheel/odom_raw'
    assert latest_joints is not None, 'no four-wheel /joint_states'


def joint_positions():
    positions = dict(zip(latest_joints.name, latest_joints.position))
    return tuple(positions[name] for name in WHEELS)


def publish_command(x=0.0, y=0.0, z=0.0, duration=0.0):
    message = Twist()
    message.linear.x = x
    message.linear.y = y
    message.angular.z = z
    if duration <= 0.0:
        publisher.publish(message)
        spin(0.05)
        return
    end = time.monotonic() + duration
    while time.monotonic() < end:
        publisher.publish(message)
        spin(0.05)


def drive(name, command, expected_signs):
    spin(0.25)
    truth_before = pose(latest_truth)
    wheel_odom_before = pose(latest_wheel_odom)
    joints_before = joint_positions()
    publish_command(*command, duration=2.0)
    publish_command(duration=0.3)
    spin(0.3)
    truth_after = pose(latest_truth)
    wheel_odom_after = pose(latest_wheel_odom)
    joints_after = joint_positions()
    physical = local_delta(truth_before, truth_after)
    raw_odom = local_delta(wheel_odom_before, wheel_odom_after)
    joint_delta = tuple(after - before for before, after in zip(joints_before, joints_after))
    for delta, sign in zip(joint_delta, expected_signs):
        assert abs(delta) > 0.05, (name, joint_delta)
        assert math.copysign(1.0, delta) == sign, (name, joint_delta, expected_signs)
    values = physical + raw_odom + joint_delta
    assert all(math.isfinite(value) for value in values), (name, values)
    return {'physical': physical, 'raw_odom': raw_odom, 'joint_delta': joint_delta}


wait_ready()
publish_command(duration=0.5)
results = {
    'x': drive('x', (0.2, 0.0, 0.0), (1.0, 1.0, 1.0, 1.0)),
    'y': drive('y', (0.0, 0.2, 0.0), (-1.0, 1.0, 1.0, -1.0)),
    'yaw': drive('yaw', (0.0, 0.0, 0.6), (-1.0, 1.0, -1.0, 1.0)),
}
assert results['x']['physical'][0] > 0.03 and abs(results['x']['physical'][0]) > abs(results['x']['physical'][1]) * 1.5, results
assert results['y']['physical'][1] > 0.03 and abs(results['y']['physical'][1]) > abs(results['y']['physical'][0]) * 1.5, results
assert results['yaw']['physical'][2] > 0.10 and math.hypot(*results['yaw']['physical'][:2]) < 0.08, results
assert results['x']['raw_odom'][0] > 0.10, results
assert results['y']['raw_odom'][1] > 0.10, results
assert results['yaw']['raw_odom'][2] > 0.20, results

# One short command becomes stale after 0.5 s. Measure drift only after the timeout has elapsed.
publish_command(0.2, 0.0, 0.0, duration=0.15)
spin(0.65)
timeout_pose = pose(latest_truth)
timeout_joints = joint_positions()
spin(2.0)
stopped_pose = pose(latest_truth)
stopped_joints = joint_positions()
post_timeout_drift = distance(timeout_pose, stopped_pose)
post_timeout_joint_motion = max(abs(after - before) for before, after in zip(timeout_joints, stopped_joints))
assert post_timeout_drift < 0.01, post_timeout_drift
assert post_timeout_joint_motion < 0.02, post_timeout_joint_motion

# Ten-second idle window simultaneously checks finite state, drift, and effective RTF.
publish_command(duration=0.3)
spin(0.5)
truth_samples.clear()
wall_start = time.monotonic()
idle_start = pose(latest_truth)
spin(10.0)
wall_elapsed = time.monotonic() - wall_start
idle_end = pose(latest_truth)
assert len(truth_samples) >= 20, len(truth_samples)
sim_elapsed = truth_samples[-1][1] - truth_samples[0][1]
rtf = sim_elapsed / (truth_samples[-1][0] - truth_samples[0][0])
idle_drift = distance(idle_start, idle_end)
assert all(math.isfinite(value) for sample in truth_samples for value in sample[2]), 'non-finite truth state'
assert idle_drift < 0.01, idle_drift
assert rtf >= 0.8, rtf

print(json.dumps({
    'motion': results,
    'post_timeout_drift': post_timeout_drift,
    'post_timeout_joint_motion': post_timeout_joint_motion,
    'idle_drift': idle_drift,
    'rtf': rtf,
    'wall_elapsed': wall_elapsed,
}, sort_keys=True))
node.destroy_node()
rclpy.shutdown()
PY

printf '%s\n' 'Kuruma gz_ros2_control acceptance smoke passed.'
