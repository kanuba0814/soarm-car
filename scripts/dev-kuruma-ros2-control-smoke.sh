#!/usr/bin/env bash
set -euo pipefail
container="$(docker compose run -d --rm -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-149}" -e IGN_PARTITION="kuruma_ros2_control_smoke_$$" ros2-dev bash -lc 'ros2 launch soarm_simulation kuruma_ros2_control.launch.py')"
cleanup() { docker stop "$container" >/dev/null 2>&1 || true; }
trap cleanup EXIT
ros='source /opt/ros/humble/setup.bash && ros2'
for _ in {1..90}; do
    if docker exec "$container" bash -lc "$ros control list_controllers 2>/dev/null | grep -q 'mecanum_drive_controller.*active'"; then break; fi
    sleep 1
done
docker exec "$container" bash -lc "$ros control list_hardware_interfaces" >/tmp/kuruma-interfaces.txt
for joint in front_left front_right rear_left rear_right; do grep -q "${joint}_wheel_joint/velocity \[available\]" /tmp/kuruma-interfaces.txt; done
docker exec "$container" bash -lc "$ros control list_controllers" | grep -q 'joint_state_broadcaster.*active'
docker exec "$container" bash -lc "$ros control list_controllers" | grep -q 'mecanum_drive_controller.*active'
docker exec "$container" bash -lc "$ros topic info /wheel/odom_raw" | grep -q 'Publisher count: 1'
docker exec "$container" bash -lc "$ros topic info /ground_truth/odom" | grep -q 'Publisher count: 1'
! docker exec "$container" bash -lc "$ros topic list | grep -qx /odom"
# The command enters through the real controller reference remapping; its timeout stops actuation.
docker exec "$container" bash -lc "$ros topic pub -r 20 --times 20 /cmd_vel geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 0.2}, angular: {z: 0.0}}}' >/dev/null"
sleep 1
docker exec "$container" bash -lc "timeout 5 $ros topic echo /ground_truth/odom --once" | grep -q 'frame_id: ground_truth'
printf '%s\n' 'Kuruma gz_ros2_control smoke passed.'
