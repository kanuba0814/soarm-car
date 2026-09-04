#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

# Build the house reference cloud from the declared map origin. Commands are
# predeclared open-loop inputs; Gazebo odometry is never read for control.
output="artifacts/fastlio_house_floorplan.pcd"
mkdir -p artifacts
ros_domain_id="${ROS_DOMAIN_ID:-91}"
ign_partition="${IGN_PARTITION:-soarm_house_map_$$}"
container="$(docker compose run -d --rm -e ROS_DOMAIN_ID="$ros_domain_id" -e IGN_PARTITION="$ign_partition" ros2-dev bash -lc 'ros2 launch soarm_simulation simulation.launch.py world:=house_floorplan ground_truth_map_tf:=false publish_sim_odometry:=false & ros2 launch fast_lio_localization mapping.launch.py use_sim_time:=true config_file:=soarm_house_mapping.yaml rviz:=false')"
cleanup() { docker stop "$container" >/dev/null 2>&1 || true; }
trap cleanup EXIT
ros='source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && ros2'
inside() { docker exec "$container" bash -lc "$ros $*"; }
for _ in {1..150}; do
    inside 'node list' 2>/dev/null | grep -qx '/laser_mapping' && break
    sleep 1
done
inside 'node list' | grep -qx '/laser_mapping'
# Anchor camera_init before motion: wait until LiDAR/IMU initialization has
# produced the first FAST-LIO pose, then run the declared open-loop arc.
for _ in {1..90}; do
    [[ "$(inside 'topic type /Odometry' 2>/dev/null || true)" == 'nav_msgs/msg/Odometry' ]] && break
    sleep 1
done
docker exec "$container" timeout 30 bash -lc "$ros topic echo /Odometry nav_msgs/msg/Odometry --once --no-daemon" >/dev/null
# This bounded arc remains in the initial open area and deliberately turns.
inside "topic pub -r 20 --times 160 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.12}, angular: {z: 0.18}}'" >/dev/null
inside "topic pub -r 20 --times 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'" >/dev/null
inside 'service call /map_save std_srvs/srv/Trigger {}' | grep -Eq 'success: true|success=True'
docker cp "$container":/workspaces/soarm-car/$output "$output"
python3 - "$output" <<'PY'
import math
import re
import struct
import sys
path = sys.argv[1]
raw = open(path, 'rb').read()
header, data = raw.split(b'DATA binary\n', 1)
points = int(re.search(rb'POINTS (\d+)', header).group(1))
assert points > 100, points
assert len(data) >= points * 32, (len(data), points)
xyz = [struct.unpack_from('<fff', data, index * 32) for index in range(points)]
assert all(math.isfinite(value) for point in xyz for value in point)
extent = [max(point[axis] for point in xyz) - min(point[axis] for point in xyz) for axis in range(3)]
assert extent[0] > 1.0 and extent[1] > 0.5 and extent[2] > 0.1, extent
print({'points': points, 'extent_xyz': extent})
PY
printf 'Generated %s anchored at the declared house start pose.\n' "$output"
