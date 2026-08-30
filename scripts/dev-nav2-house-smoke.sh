#!/usr/bin/env bash
set -euo pipefail

# Consumer smoke: installed Nav2 drives the native Fortress Ackermann model to a house goal.
container_name="${SOARM_TEST_CONTAINER:-soarm-nav2-house-test-$$}"
test_image="${SOARM_TEST_IMAGE:-soarm-car-nav2-ros2-dev:latest}"
ros_domain_id="${ROS_DOMAIN_ID:-97}"
ign_partition="${IGN_PARTITION:-${container_name}-ign}"
output_dir="$(mktemp -d /tmp/soarm-nav2-house-smoke.XXXXXX)"
succeeded=false
cleanup() {
    docker stop "$container_name" >/dev/null 2>&1 || true
    docker logs "$container_name" >"$output_dir/container.log" 2>&1 || true
    if [[ "${SOARM_KEEP_CONTAINER:-false}" != true ]]; then
        docker rm "$container_name" >/dev/null 2>&1 || true
    fi
    if "$succeeded"; then
        rm -rf "$output_dir"
    else
        printf 'Nav2 house smoke failed; logs retained in %s\n' "$output_dir" >&2
    fi
}
trap cleanup EXIT

python3 - <<'PY'
from pathlib import Path

params = Path('ros2_ws/src/soarm_navigation/config/house_nav2.yaml').read_text()
tree = Path('ros2_ws/src/soarm_navigation/behavior_trees/navigate_ackermann.xml').read_text()
through_poses_tree = Path('ros2_ws/src/soarm_navigation/behavior_trees/navigate_through_poses_ackermann.xml').read_text()
assert 'motion_model_for_search: DUBIN' in params
assert 'minimum_turning_radius: 0.8' in params
assert 'use_rotate_to_heading: false' in params
assert 'behavior_plugins: [backup, drive_on_heading, wait]' in params
assert 'topic: /mid360/points' in params and 'marking: true' in params and 'clearing: true' in params
assert 'observation_persistence: 0.5' in params and 'max_obstacle_height: 1.5' in params
assert '<Spin' not in tree and '<Rotate' not in tree
assert '<Spin' not in through_poses_tree and '<Rotate' not in through_poses_tree
PY

docker image inspect "$test_image" >/dev/null
container="$(docker run -d \
    --name "$container_name" \
    --network host \
    --ipc host \
    --device /dev/dri \
    --group-add "${VIDEO_GID:-39}" \
    --group-add "${RENDER_GID:-105}" \
    --security-opt label=disable \
    -e ROS_DOMAIN_ID="$ros_domain_id" \
    -e IGN_PARTITION="$ign_partition" \
    -e XDG_RUNTIME_DIR=/tmp/runtime-root \
    -e LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}" \
    -v "$(pwd -P):/feature:ro" \
    -w /feature \
    --entrypoint bash \
    "$test_image" -lc '
        source /opt/ros/humble/setup.bash
        source /workspaces/soarm-car/install/setup.bash
        colcon --log-base /tmp/nav2_house/log build \
            --base-paths /feature/ros2_ws/src/soarm_simulation /feature/ros2_ws/src/soarm_navigation \
            --build-base /tmp/nav2_house/build \
            --install-base /tmp/nav2_house/install \
            --symlink-install
        source /tmp/nav2_house/install/setup.bash
        ros2 launch soarm_navigation house_nav2.launch.py
    ')"

ros_setup='source /opt/ros/humble/setup.bash && source /workspaces/soarm-car/install/setup.bash && source /tmp/nav2_house/install/setup.bash &&'
ready=false
for _ in {1..150}; do
    if docker exec "$container_name" bash -lc "$ros_setup ros2 node list" >"$output_dir/nodes.txt" 2>&1 \
        && grep -qx '/planner_server' "$output_dir/nodes.txt" \
        && grep -qx '/controller_server' "$output_dir/nodes.txt" \
        && grep -qx '/bt_navigator' "$output_dir/nodes.txt" \
        && grep -qx '/behavior_server' "$output_dir/nodes.txt" \
        && grep -qx '/velocity_smoother' "$output_dir/nodes.txt" \
        && ! grep -q amcl "$output_dir/nodes.txt"; then
        ready=true
        break
    fi
    sleep 1
done
$ready || { cat "$output_dir/nodes.txt" >&2; exit 1; }

docker exec -i "$container_name" bash -lc "$ros_setup python3 -" >"$output_dir/action.txt" <<'PY'
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient

latest_odom = None

def on_odom(message):
    global latest_odom
    latest_odom = message

rclpy.init()
node = rclpy.create_node('house_nav2_smoke_client')
subscription = node.create_subscription(Odometry, '/odom', on_odom, 20)
client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
assert client.wait_for_server(timeout_sec=30.0), 'NavigateToPose server unavailable'
end = time.monotonic() + 15.0
while latest_odom is None and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
assert latest_odom is not None, 'no /odom before goal'
start = latest_odom.pose.pose.position
# The pose is map-frame ground truth. It is two metres north of the spawn and
# requires the initially east-facing car to follow a curved, non-holonomic path.
goal = NavigateToPose.Goal()
goal.pose = PoseStamped()
goal.pose.header.frame_id = 'map'
goal.pose.header.stamp = node.get_clock().now().to_msg()
goal.pose.pose.position.x = 0.0
goal.pose.pose.position.y = 2.0
goal.pose.pose.orientation.w = 1.0
assert math.hypot(goal.pose.pose.position.x - start.x, goal.pose.pose.position.y - start.y) > 1.0
send_future = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, send_future, timeout_sec=15.0)
goal_handle = send_future.result()
assert goal_handle is not None and goal_handle.accepted, 'NavigateToPose rejected the goal'
result_future = goal_handle.get_result_async()
rclpy.spin_until_future_complete(node, result_future, timeout_sec=120.0)
result = result_future.result()
assert result is not None, 'NavigateToPose timed out'
assert result.status == 4, f'NavigateToPose status {result.status}, expected SUCCEEDED (4)'
# Continue spinning after success so the consumer check sees the settled native odometry.
settle_end = time.monotonic() + 1.5
while time.monotonic() < settle_end:
    rclpy.spin_once(node, timeout_sec=0.1)
final = latest_odom.pose.pose.position
error = math.hypot(final.x - goal.pose.pose.position.x, final.y - goal.pose.pose.position.y)
linear_speed = math.hypot(latest_odom.twist.twist.linear.x, latest_odom.twist.twist.linear.y)
angular_speed = abs(latest_odom.twist.twist.angular.z)
assert error <= 0.35, f'final position error {error:.3f} m'
assert linear_speed <= 0.05 and angular_speed <= 0.10, (linear_speed, angular_speed)
print(f'start=({start.x:.3f},{start.y:.3f}) final=({final.x:.3f},{final.y:.3f}) error={error:.3f} m speed=({linear_speed:.3f},{angular_speed:.3f})')
node.destroy_node()
rclpy.shutdown()
PY

succeeded=true
printf '%s\n' "Nav2 house consumer smoke passed in isolated container $container_name."
