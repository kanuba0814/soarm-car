#!/usr/bin/env bash
set -euo pipefail

# Consumer smoke: installed Nav2 drives the native Fortress mecanum model to a house goal.
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

import yaml

params = yaml.safe_load(Path('ros2_ws/src/soarm_navigation/config/house_nav2.yaml').read_text())
local_source = params['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']['mid360']
tree = Path('ros2_ws/src/soarm_navigation/behavior_trees/navigate_holonomic.xml').read_text()
through_poses_tree = Path('ros2_ws/src/soarm_navigation/behavior_trees/navigate_through_poses_holonomic.xml').read_text()
assert params['planner_server']['ros__parameters']['GridBased']['plugin'] == 'nav2_smac_planner/SmacPlanner2D'
assert params['controller_server']['ros__parameters']['FollowPath']['plugin'] == 'dwb_core::DWBLocalPlanner'
assert params['velocity_smoother']['ros__parameters']['max_velocity'][1] == 0.4
assert params['behavior_server']['ros__parameters']['behavior_plugins'] == ['backup', 'drive_on_heading', 'wait']
assert local_source['topic'] == '/mid360/points' and local_source['marking'] and local_source['clearing']
assert local_source['obstacle_min_range'] == 0.60
assert 0.45 < local_source['obstacle_min_range'] < local_source['obstacle_max_range']
assert local_source['observation_persistence'] == 0.5 and local_source['max_obstacle_height'] == 1.5
# The smoke target is map-free with one metre of static-wall clearance, leaving
# room for the 0.31 m square footprint.
pgm = Path('ros2_ws/src/soarm_simulation/maps/house_floorplan.pgm').read_bytes().split(b'\n255\n', 1)[1]
width, height, resolution, origin_x, origin_y = 480, 435, 0.05, -12.0, -10.875
def cell_value(x, y):
    column = int((x - origin_x) / resolution)
    row = height - 1 - int((y - origin_y) / resolution)
    return pgm[row * width + column]
assert cell_value(1.5, 1.5) == 254
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
        && ! grep -q amcl "$output_dir/nodes.txt" \
        && docker exec "$container_name" bash -lc "$ros_setup ros2 param get /local_costmap/local_costmap voxel_layer.mid360.obstacle_min_range" | grep -q '0.6'; then
        ready=true
        break
    fi
    sleep 1
done
$ready || { cat "$output_dir/nodes.txt" >&2; exit 1; }

docker exec -i "$container_name" bash -lc "$ros_setup python3 -" >"$output_dir/action.txt" <<'PY'
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient

latest_odom = None
latest_global_plan = None
latest_local_plan = None
latest_costmap = None
max_occupied_cells = 0
odom_samples = []
feedback_samples = []


def yaw_from_quaternion(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))


def on_odom(message):
    global latest_odom
    latest_odom = message
    position = message.pose.pose.position
    odom_samples.append({
        'stamp': message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
        'x': round(position.x, 3),
        'y': round(position.y, 3),
        'yaw': round(yaw_from_quaternion(message.pose.pose.orientation), 3),
        'linear_x': round(message.twist.twist.linear.x, 3),
        'linear_y': round(message.twist.twist.linear.y, 3),
        'angular_z': round(message.twist.twist.angular.z, 3),
    })


def on_global_plan(message):
    global latest_global_plan
    latest_global_plan = message


def on_local_plan(message):
    global latest_local_plan
    latest_local_plan = message


def on_costmap(message):
    global latest_costmap, max_occupied_cells
    latest_costmap = message
    max_occupied_cells = max(max_occupied_cells, sum(value >= 100 for value in message.data))


def path_geometry(path):
    if path is None:
        return []
    poses = path.poses
    stride = max(1, len(poses) // 12)
    sampled = poses[::stride]
    if poses and sampled[-1] is not poses[-1]:
        sampled.append(poses[-1])
    return [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in sampled]


def costmap_summary(costmap, odom):
    if costmap is None or odom is None:
        return None
    metadata = costmap.metadata
    position = odom.pose.pose.position
    column = int((position.x - metadata.origin.position.x) / metadata.resolution)
    row = int((position.y - metadata.origin.position.y) / metadata.resolution)
    index = row * metadata.size_x + column
    value = costmap.data[index] if 0 <= column < metadata.size_x and 0 <= row < metadata.size_y else None
    occupied = sum(value >= 100 for value in costmap.data)
    return {
        'frame': costmap.header.frame_id,
        'resolution': metadata.resolution,
        'size': [metadata.size_x, metadata.size_y],
        'origin': [round(metadata.origin.position.x, 3), round(metadata.origin.position.y, 3)],
        'robot_cell': [column, row, value],
        'occupied_cells': occupied,
    }


def on_feedback(message):
    feedback = message.feedback
    feedback_samples.append({
        'distance_remaining': round(feedback.distance_remaining, 3),
        'navigation_time': round(feedback.navigation_time.sec + feedback.navigation_time.nanosec / 1e9, 3),
    })

rclpy.init()
node = rclpy.create_node('house_nav2_smoke_client')
subscription = node.create_subscription(Odometry, '/odom', on_odom, 20)
node.create_subscription(Path, '/plan', on_global_plan, 10)
node.create_subscription(Path, '/local_plan', on_local_plan, 10)
node.create_subscription(Costmap, '/local_costmap/costmap_raw', on_costmap, 10)
client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
assert client.wait_for_server(timeout_sec=30.0), 'NavigateToPose server unavailable'
end = time.monotonic() + 15.0
while latest_odom is None and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
assert latest_odom is not None, 'no /odom before goal'
start = latest_odom.pose.pose.position
start_yaw = yaw_from_quaternion(latest_odom.pose.pose.orientation)
goal = NavigateToPose.Goal()
goal.pose = PoseStamped()
goal.pose.header.frame_id = 'map'
goal.pose.header.stamp = node.get_clock().now().to_msg()
# This map-free diagonal goal is 2.12 m away and needs a holonomic approach;
# its pi/4 final heading exercises position and final-yaw tracking in the open map area.
goal.pose.pose.position.x = 1.5
goal.pose.pose.position.y = 1.5
goal.pose.pose.orientation.z = math.sin(math.pi / 8.0)
goal.pose.pose.orientation.w = math.cos(math.pi / 8.0)
assert math.hypot(goal.pose.pose.position.x - start.x, goal.pose.pose.position.y - start.y) > 1.0
send_future = client.send_goal_async(goal, feedback_callback=on_feedback)
rclpy.spin_until_future_complete(node, send_future, timeout_sec=15.0)
goal_handle = send_future.result()
assert goal_handle is not None and goal_handle.accepted, 'NavigateToPose rejected the goal'
result_future = goal_handle.get_result_async()
rclpy.spin_until_future_complete(node, result_future, timeout_sec=120.0)
result = result_future.result()
assert result is not None, 'NavigateToPose timed out'
# Keep diagnostics ahead of pass/fail assertions so an abort retains the collision geometry.
settle_end = time.monotonic() + 1.5
while time.monotonic() < settle_end:
    rclpy.spin_once(node, timeout_sec=0.1)
final = latest_odom.pose.pose.position
error = math.hypot(final.x - goal.pose.pose.position.x, final.y - goal.pose.pose.position.y)
linear_speed = math.hypot(latest_odom.twist.twist.linear.x, latest_odom.twist.twist.linear.y)
angular_speed = abs(latest_odom.twist.twist.angular.z)
yaw_changes = [abs(math.atan2(math.sin(sample['yaw'] - start_yaw), math.cos(sample['yaw'] - start_yaw)))
               for sample in odom_samples]
max_yaw_change = max(yaw_changes, default=0.0)
max_lateral_displacement = max((abs(sample['y'] - start.y) for sample in odom_samples), default=0.0)
print(json.dumps({
    'result_status': result.status,
    'start': [round(start.x, 3), round(start.y, 3)],
    'final': [round(final.x, 3), round(final.y, 3)],
    'final_error': round(error, 3),
    'final_speed': [round(linear_speed, 3), round(angular_speed, 3)],
    'max_yaw_change': round(max_yaw_change, 3),
    'max_lateral_displacement': round(max_lateral_displacement, 3),
    'max_local_costmap_occupied_cells': max_occupied_cells,
    'global_plan': path_geometry(latest_global_plan),
    'local_plan': path_geometry(latest_local_plan),
    'feedback_tail': feedback_samples[-8:],
    'odom_tail': odom_samples[-12:],
    'local_costmap': costmap_summary(latest_costmap, latest_odom),
}, sort_keys=True))
assert result.status == 4, f'NavigateToPose status {result.status}, expected SUCCEEDED (4)'
assert error <= 0.35, f'final position error {error:.3f} m'
assert max_yaw_change >= 0.35, f'path did not exercise a material turn: {max_yaw_change:.3f} rad'
assert max_lateral_displacement >= 0.5, f'path lateral displacement too small: {max_lateral_displacement:.3f} m'
assert latest_costmap is not None, 'no local costmap received'
assert max_occupied_cells > 0, 'local VoxelLayer never retained an occupied obstacle cell'
assert linear_speed <= 0.05 and angular_speed <= 0.10, (linear_speed, angular_speed)
node.destroy_node()
rclpy.shutdown()
PY

succeeded=true
printf '%s\n' "Nav2 house consumer smoke passed in isolated container $container_name."
