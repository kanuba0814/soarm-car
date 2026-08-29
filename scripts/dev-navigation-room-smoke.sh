#!/usr/bin/env bash
set -euo pipefail
# Consumer smoke: Fortress must load the room and produce obstacle returns and physical odometry.
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
world = ET.parse('ros2_ws/src/soarm_simulation/worlds/navigation_room.sdf').getroot()
assert world.find("./world/include/uri[.='model://navigation_room']") is not None
assert world.find("./world/include/uri[.='model://senior_akm']") is not None
room = ET.parse('ros2_ws/src/soarm_simulation/models/navigation_room/model.sdf').getroot()
for collision in ('north_wall', 'south_wall_west', 'east_wall', 'west_wall', 'l_wall_vertical', 'l_wall_horizontal', 'cylinder', 'small_box', 'large_box'):
    assert room.find(f".//collision[@name='{collision}']") is not None, collision
pgm = Path('ros2_ws/src/soarm_simulation/maps/navigation_room.pgm').read_bytes()
assert pgm.startswith(b'P5\n') and b'160 120\n255\n' in pgm
assert 'resolution: 0.05' in Path('ros2_ws/src/soarm_simulation/maps/navigation_room.yaml').read_text()
assert 'origin: [-4.0, -3.0, 0.0]' in Path('ros2_ws/src/soarm_simulation/maps/navigation_room.yaml').read_text()
PY
ros_domain_id="${ROS_DOMAIN_ID:-86}"
ign_partition="${IGN_PARTITION:-soarm_navigation_smoke_$$}"
container="$(docker compose run -d --rm -e ROS_DOMAIN_ID="$ros_domain_id" -e IGN_PARTITION="$ign_partition" ros2-dev bash -lc 'ros2 launch soarm_simulation simulation.launch.py world:=navigation_room')"
cleanup() { docker stop "$container" >/dev/null 2>&1 || true; }
trap cleanup EXIT
ros_command='source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && ros2'
for _ in {1..120}; do
    if docker exec "$container" bash -lc "test -f /tmp/soarm_ros2/install/setup.bash && $ros_command topic info /mid360/points | grep -q 'Publisher count: 1'"; then break; fi
    sleep 1
done
inside() { docker exec "$container" bash -lc "$ros_command $*"; }
inside_timeout() { local seconds=$1 command; shift; printf -v command '%q ' "$@"; docker exec "$container" bash -lc "source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && timeout $seconds $command"; }
inside_timeout 15 ros2 topic echo /mid360/points --once --qos-reliability best_effort >/tmp/navigation-room-points.yaml
inside_timeout 10 ros2 topic echo /gazebo/odom --once >/tmp/navigation-room-odom-before.yaml
inside "topic pub -r 20 --times 60 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}'" >/dev/null
inside_timeout 10 ros2 topic echo /gazebo/odom --once >/tmp/navigation-room-odom-drive.yaml
inside "topic pub -r 20 --times 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'" >/dev/null
sleep 1
inside_timeout 10 ros2 topic echo /gazebo/odom --once >/tmp/navigation-room-odom-stop.yaml
python3 - <<'PY'
import re
points = open('/tmp/navigation-room-points.yaml').read()
width = re.search(r'width: (\d+)', points)
data = re.search(r'data:\n\s*- ', points)
assert width and int(width.group(1)) > 100 and data, points[:1000]
def x_position(path):
    text = open(path).read()
    match = re.search(r'position:\n      x: ([^\n]+)', text)
    assert match, text
    return float(match.group(1))
def vx(path):
    text = open(path).read()
    match = re.search(r'linear:\n      x: ([^\n]+)', text)
    assert match, text
    return float(match.group(1))
before = x_position('/tmp/navigation-room-odom-before.yaml')
drive = x_position('/tmp/navigation-room-odom-drive.yaml')
assert abs(drive - before) > 0.1, (before, drive)
assert abs(vx('/tmp/navigation-room-odom-stop.yaml')) < 1e-5
PY
printf '%s\n' 'Navigation room Fortress consumer smoke passed.'
