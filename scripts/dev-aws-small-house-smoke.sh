#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
manifest="$root/ros2_ws/src/soarm_simulation/config/aws_small_house_baseline.json"
baseline_dir="${AWS_SMALL_HOUSE_DIR:-$root/artifacts/baselines/aws_small_house_world}"
container_name="${SOARM_TEST_CONTAINER:-soarm-aws-small-house-$$}"
test_image="${SOARM_TEST_IMAGE:-soarm-car-ros2-dev:hm3d}"
ros_domain_id="${ROS_DOMAIN_ID:-119}"
ign_partition="${IGN_PARTITION:-${container_name}-ign}"
temporary="$(mktemp -d /tmp/soarm-aws-small-house.XXXXXX)"

cleanup() {
    docker rm -f "$container_name" >/dev/null 2>&1 || true
    rm -rf "$temporary"
}
trap cleanup EXIT

readarray -t baseline < <(python3 - "$manifest" <<'PY'
import json, sys
config = json.load(open(sys.argv[1]))
print(config['repository'])
print(config['branch'])
print(config['commit'])
PY
)
if [[ ! -d "$baseline_dir/.git" ]]; then
    mkdir -p "$(dirname "$baseline_dir")"
    git clone --branch "${baseline[1]}" "${baseline[0]}" "$baseline_dir"
    git -C "$baseline_dir" checkout --detach "${baseline[2]}"
fi
actual_commit="$(git -C "$baseline_dir" rev-parse HEAD)"
[[ "$actual_commit" == "${baseline[2]}" ]] || {
    printf 'AWS baseline must be at %s, found %s in %s\n' "${baseline[2]}" "$actual_commit" "$baseline_dir" >&2
    exit 1
}

python3 - "$baseline_dir/worlds/small_house.world" "$temporary/aws_small_house_baseline.sdf" <<'PY'
import sys
import xml.etree.ElementTree as ET
source, destination = sys.argv[1:]
tree = ET.parse(source)
world = tree.getroot().find('world')
world.set('name', 'aws_small_house_baseline')
include = ET.SubElement(world, 'include')
ET.SubElement(include, 'uri').text = 'model://senior_akm'
ET.SubElement(include, 'name').text = 'senior_akm'
ET.SubElement(include, 'pose').text = '0 0 0.08 0 0 0'
tree.write(destination, encoding='utf-8', xml_declaration=True)
PY

docker run -d --rm --name "$container_name" --network host --ipc host \
    --device /dev/dri --group-add "${VIDEO_GID:-39}" --group-add "${RENDER_GID:-105}" \
    --security-opt label=disable -e ROS_DOMAIN_ID="$ros_domain_id" -e IGN_PARTITION="$ign_partition" \
    -e XDG_RUNTIME_DIR=/tmp/runtime-root -e LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}" \
    -v "$root:/feature:ro" -v "$baseline_dir:/baseline:ro" -v "$temporary:/runtime:ro" \
    -w /feature --entrypoint bash "$test_image" -lc '
        mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root
        source /opt/ros/humble/setup.bash
        source /workspaces/soarm-car/install/setup.bash
        colcon --log-base /tmp/aws_baseline/log build \
            --base-paths ros2_ws/src/soarm_description ros2_ws/src/soarm_astra ros2_ws/src/soarm_simulation \
            --build-base /tmp/aws_baseline/build --install-base /tmp/aws_baseline/install --symlink-install
        source /tmp/aws_baseline/install/setup.bash
        export IGN_GAZEBO_RESOURCE_PATH=/baseline/models:/tmp/aws_baseline/install/soarm_simulation/share/soarm_simulation/models
        export GZ_SIM_RESOURCE_PATH="$IGN_GAZEBO_RESOURCE_PATH"
        ign gazebo -r -s /runtime/aws_small_house_baseline.sdf &
        ros2 run ros_gz_bridge parameter_bridge \
            /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
            /gazebo/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
            /camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
            /camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image \
            /mid360/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked &
        ros2 run soarm_simulation sim_adapter --ros-args \
            -p policy_file:=/tmp/aws_baseline/install/soarm_simulation/share/soarm_simulation/config/mission_policy.yaml &
        wait -n
    ' >/dev/null

ready=false
for _ in {1..180}; do
    if docker exec "$container_name" bash -lc "source /opt/ros/humble/setup.bash; source /workspaces/soarm-car/install/setup.bash; source /tmp/aws_baseline/install/setup.bash; ROS_DOMAIN_ID=$ros_domain_id ros2 topic list" >"$temporary/topics" 2>/dev/null \
        && grep -qx /camera/image "$temporary/topics" \
        && grep -qx /camera/depth_image "$temporary/topics" \
        && grep -qx /mid360/points "$temporary/topics"; then
        ready=true
        break
    fi
    sleep 1
done
$ready || { docker logs "$container_name" >&2; exit 1; }

docker exec -i "$container_name" bash -lc "source /opt/ros/humble/setup.bash; source /workspaces/soarm-car/install/setup.bash; source /tmp/aws_baseline/install/setup.bash; export ROS_DOMAIN_ID=$ros_domain_id; timeout 40 python3 -" <<'PY'
import json, math, struct, time
import numpy as np
import rclpy
from sensor_msgs.msg import Image, PointCloud2
latest = {}
rclpy.init()
node = rclpy.create_node('aws_small_house_baseline_probe')
for topic, message_type, key in [('/camera/image', Image, 'rgb'), ('/camera/depth_image', Image, 'depth'), ('/mid360/points', PointCloud2, 'cloud')]:
    node.create_subscription(message_type, topic, lambda message, key=key: latest.__setitem__(key, message), 10)
end = time.monotonic() + 35
while len(latest) < 3 and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
assert len(latest) == 3, 'baseline sensor messages incomplete'
rgb = latest['rgb']
depth = latest['depth']
cloud = latest['cloud']
colors = np.frombuffer(bytes(rgb.data), np.uint8)
depths = np.frombuffer(bytes(depth.data), np.float32)
fields = {field.name: field.offset for field in cloud.fields}
finite_points = 0
for index in range(0, cloud.width * cloud.height, max(1, cloud.width * cloud.height // 3000)):
    row, column = divmod(index, cloud.width)
    offset = row * cloud.row_step + column * cloud.point_step
    point = [struct.unpack_from('<f', cloud.data, offset + fields[name])[0] for name in ('x', 'y', 'z')]
    finite_points += int(all(math.isfinite(value) for value in point))
summary = {'rgb_unique_values': int(len(np.unique(colors))), 'depth_finite_ratio': float(np.isfinite(depths).mean()), 'finite_lidar_samples': finite_points}
print(json.dumps(summary, sort_keys=True))
assert summary['rgb_unique_values'] > 32
assert summary['depth_finite_ratio'] > 0.2
assert finite_points > 100
PY

models="$(docker exec "$container_name" bash -lc "IGN_PARTITION=$ign_partition ign model --list")"
grep -q senior_akm <<<"$models"
! docker logs "$container_name" 2>&1 | grep -Eqi 'invalid mesh|failed to load geometry|missing texture|segmentation fault'
printf 'AWS Small House baseline smoke passed at %s\n' "$actual_commit"
