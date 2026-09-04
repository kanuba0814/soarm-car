#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET

model = ET.parse('ros2_ws/src/soarm_simulation/models/kuruma/model.sdf').getroot()
sensor = model.find(".//sensor[@name='camera']")
assert sensor is not None and sensor.attrib['type'] == 'rgbd_camera'
assert sensor.findtext('topic') == '/camera'
assert sensor.findtext('update_rate') == '30'
assert sensor.findtext('camera/image/width') == '640'
assert sensor.findtext('camera/image/height') == '480'
launch = Path('ros2_ws/src/soarm_simulation/soarm_simulation/launch/simulation.launch.py').read_text()
for topic in ('/simulation/astra/color/image', '/simulation/astra/depth/image',
              '/simulation/astra/camera_info'):
    assert topic in launch
adapter = Path('ros2_ws/src/soarm_astra/soarm_astra/sim_adapter.py').read_text()
for topic in ('/camera/color/image_raw', '/camera/depth/image_raw',
              '/camera/color/camera_info', '/camera/depth/camera_info'):
    assert topic in adapter
assert "output.encoding = '16UC1'" in adapter
PY

ros_domain_id="${ROS_DOMAIN_ID:-$((100 + RANDOM % 100))}"
ign_partition="${IGN_PARTITION:-astra_sim_smoke_$$}"
container="soarm-astra-sim-smoke-$$"
docker compose run -d --name "$container" --quiet \
    -e ROS_DOMAIN_ID="$ros_domain_id" -e ROS_LOCALHOST_ONLY=1 \
    -e IGN_PARTITION="$ign_partition" ros2-dev bash -lc \
    'ros2 launch soarm_simulation simulation.launch.py'
cleanup() { docker rm -f "$container" >/dev/null 2>&1 || true; }
trap cleanup EXIT
setup='source /opt/ros/humble/setup.bash && source /opt/astra_ros2/setup.bash && source /tmp/soarm_ros2/install/setup.bash'
for _ in {1..120}; do
    if [[ "$(docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null || true)" != true ]]; then
        docker logs "$container" >&2 || true
        echo 'Gazebo container exited before the Astra adapter became ready.' >&2
        exit 1
    fi
    if docker exec "$container" bash -lc "test -f /tmp/soarm_ros2/install/setup.bash && $setup && nodes=\$(ros2 node list 2>/dev/null) && [[ \$nodes == *'/astra_sim_adapter'* && \$nodes == *'/camera/point_cloud_xyz'* ]]"; then
        break
    fi
    sleep 1
done

docker exec "$container" bash -lc "$setup && nodes=\$(ros2 node list) && [[ \$nodes == *'/astra_sim_adapter'* && \$nodes == *'/camera/point_cloud_xyz'* ]]"
docker exec "$container" bash -lc "$setup && python3 /workspaces/soarm-car/scripts/astra-contract-check.py --timeout 25"
printf '%s\n' 'Gazebo Astra-compatible payload contract passed.'
