#!/usr/bin/env bash
set -euo pipefail
# Consumer smoke for the Fortress PointCloudPacked -> CustomMsg -> FAST-LIO seam.
# It intentionally records the optional Open3D localization dependency as a block when absent.
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
model = ET.parse('ros2_ws/src/soarm_simulation/models/kuruma/model.sdf').getroot()
livox = model.find("./model/link[@name='livox_frame']")
assert livox is not None and livox.find("sensor[@name='livox_imu']") is not None
launch = Path('ros2_ws/src/soarm_simulation/soarm_simulation/launch/simulation.launch.py').read_text()
assert 'gz.msgs.PointCloudPacked' in launch and '/gazebo/livox_imu' in launch
config = yaml.safe_load(Path('vendor/localization/src/FAST_LIO_LOCALIZATION_ROS2/config/soarm_mid360_sim.yaml').read_text())['/**']['ros__parameters']
assert config['common']['lid_topic'] == '/livox/lidar'
assert config['common']['imu_topic'] == '/livox/imu'
assert config['preprocess']['lidar_type'] == 1 and config['preprocess']['scan_line'] == 4
assert config['preprocess']['scan_rate'] == 10
assert config['mapping']['extrinsic_est_en'] is False
PY
# Isolate this simulation from any GUI/dev container already publishing the same
# topic names with a different /clock. Mixed simulation clocks make FAST-LIO
# correctly reject data as looped-back and unsynchronised.
ros_domain_id="${ROS_DOMAIN_ID:-87}"
ign_partition="${IGN_PARTITION:-soarm_fastlio_smoke_$$}"
mkdir -p artifacts
container="$(docker compose run -d --rm -e ROS_DOMAIN_ID="$ros_domain_id" -e IGN_PARTITION="$ign_partition" ros2-dev bash -lc 'ros2 launch soarm_simulation simulation.launch.py world:=navigation_room & ros2 launch fast_lio_localization mapping.launch.py use_sim_time:=true config_file:=soarm_mid360_sim.yaml rviz:=false')"
cleanup() { docker stop "$container" >/dev/null 2>&1 || true; }
trap cleanup EXIT
ros='source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && ros2'
inside() { docker exec "$container" bash -lc "$ros $*"; }
bounded_echo() {
    local seconds=$1 topic=$2 output=$3 qos=${4:-}
    docker exec "$container" bash -lc "$ros topic echo '$topic' --once $qos > /tmp/soarm-bounded-echo 2>&1 & pid=\$!; for ((i=0; i<$seconds; i++)); do if ! kill -0 \$pid 2>/dev/null; then wait \$pid; status=\$?; cat /tmp/soarm-bounded-echo; exit \$status; fi; sleep 1; done; kill \$pid 2>/dev/null; wait \$pid 2>/dev/null; cat /tmp/soarm-bounded-echo; exit 124" >"$output"
}
ready=false
for _ in {1..120}; do
    if docker exec "$container" bash -lc "output=\$($ros topic info /livox/lidar); [[ \$output == *'Publisher count: 1'* ]] && [[ \$output == *'livox_ros_driver2/msg/CustomMsg'* ]] && [[ \$($ros node list) == *'/laser_mapping'* ]]" 2>/dev/null; then
        ready=true
        break
    fi
    sleep 1
done
$ready || { docker logs "$container" >&2; exit 1; }
docker exec "$container" test -f /tmp/soarm_ros2/install/setup.bash
for topic in /livox/imu /Odometry /cloud_registered /Laser_map; do
    docker exec "$container" bash -lc "output=\$($ros topic info $topic); [[ \$output =~ 'Publisher count: '[1-9] ]]"
done
bounded_echo 20 /mid360/points /tmp/fastlio-mid360.yaml '--qos-reliability best_effort'
bounded_echo 20 /livox/lidar /tmp/fastlio-livox.yaml '--qos-reliability best_effort'
bounded_echo 20 /livox/imu /tmp/fastlio-imu.yaml '--qos-reliability best_effort'
# Safe room trajectory and zero command after it. --times keeps publishers
# finite inside the container; killing docker exec alone would orphan them.
inside "topic pub -r 20 --times 120 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}, angular: {z: 0.18}}'" >/dev/null
inside "topic pub -r 20 --times 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'" >/dev/null
bounded_echo 30 /Odometry /tmp/fastlio-odom.yaml
bounded_echo 20 /gazebo/odom /tmp/fastlio-gazebo-odom.yaml
map_save_response="$(inside 'service call /map_save std_srvs/srv/Trigger {}')"
[[ "$map_save_response" == *'success=True'* || "$map_save_response" == *'success: true'* ]]
docker cp "$container":/workspaces/soarm-car/artifacts/fastlio_navigation_room.pcd /tmp/fastlio_navigation_room.pcd
python3 - <<'PY'
import re
mid360 = open('/tmp/fastlio-mid360.yaml').read()
assert 'width: ' in mid360 and int(re.search(r'width: (\d+)', mid360).group(1)) > 100
assert 'frame_id: livox_frame' in mid360
livox = open('/tmp/fastlio-livox.yaml').read()
point_num = int(re.search(r'point_num: (\d+)', livox).group(1))
stamp = re.search(r'stamp:\n    sec: (\d+)\n    nanosec: (\d+)', livox)
timebase = int(re.search(r'timebase: (\d+)', livox).group(1))
assert stamp and timebase == int(stamp.group(1)) * 1_000_000_000 + int(stamp.group(2))
assert point_num > 100, point_num
points = re.findall(r'offset_time: (\d+).*?x: ([^\n]+).*?y: ([^\n]+).*?z: ([^\n]+).*?line: (\d+)', livox, re.S)
assert len(points) > 100, len(points)
offsets = [int(point[0]) for point in points]
assert offsets == sorted(offsets) and offsets[-1] > offsets[0]
# ros2 topic echo truncates arrays after 128 entries, so only validate the
# observed prefix here; point_num above validates the complete message size.
assert offsets[-1] <= 100_000_000
assert set(int(point[4]) for point in points).issubset({0, 1, 2, 3})
assert 'frame_id: livox_frame' in open('/tmp/fastlio-imu.yaml').read()
def xy_position(path):
    text = open(path).read()
    match = re.search(r'position:\n      x: ([^\n]+)\n      y: ([^\n]+)', text)
    assert match, path
    return float(match.group(1)), float(match.group(2))
fastlio_xy = xy_position('/tmp/fastlio-odom.yaml')
gazebo_xy = xy_position('/tmp/fastlio-gazebo-odom.yaml')
assert fastlio_xy[0] ** 2 + fastlio_xy[1] ** 2 > 0.01, fastlio_xy
assert gazebo_xy[0] > 0.1, gazebo_xy
assert fastlio_xy[0] * gazebo_xy[0] > 0.0, (fastlio_xy, gazebo_xy)
pcd = open('/tmp/fastlio_navigation_room.pcd', 'rb').read()
header, data = pcd.split(b'DATA binary\n', 1)
point_count = int(re.search(rb'POINTS (\d+)', header).group(1))
assert point_count > 100 and len(data) >= point_count * 32
# FAST-LIO writes eight float32 fields per point. Validate that the resulting
# map is genuinely 3D rather than the old LaserScan z=0 conversion.
import struct
z_values = [struct.unpack_from('<f', data, index * 32 + 8)[0] for index in range(point_count)]
assert max(z_values) - min(z_values) > 0.1
PY
docker exec "$container" bash -lc "$ros -h >/dev/null && python3 -c 'import open3d, tf_transformations'"
printf '%s\n' 'FAST-LIO navigation-room consumer smoke passed.'
