#!/usr/bin/env bash
set -euo pipefail

mode="${1:-verify}"
[[ "$mode" == record || "$mode" == verify ]] || { echo "usage: $0 [record|verify]" >&2; exit 2; }
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
test_image="${SOARM_TEST_IMAGE:-soarm-car-ros2-dev:hm3d}"
output_dir="${SOARM_REGRESSION_DIR:-$root/regression}"
mkdir -p "$output_dir"

for index in 0 1; do
    world="apartment_$([[ $index == 0 ]] && echo a || echo b)"
    domain_id=$((120 + index))
    container="soarm-${world}-regression-$$"
    partition="${container}-ign"
    temporary="$(mktemp -d /tmp/soarm-${world}-regression.XXXXXX)"
    cleanup_world() {
        docker rm -f "$container" >/dev/null 2>&1 || true
        rm -rf "$temporary"
    }
    trap cleanup_world EXIT

    docker run -d --rm --name "$container" --network host --ipc host \
        --device /dev/dri --group-add "${VIDEO_GID:-39}" --group-add "${RENDER_GID:-105}" \
        --security-opt label=disable -e ROS_DOMAIN_ID="$domain_id" -e IGN_PARTITION="$partition" \
        -e XDG_RUNTIME_DIR=/tmp/runtime-root -e LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}" \
        -v "$root:/feature:ro" -w /feature --entrypoint bash "$test_image" -lc "
            mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root
            source /opt/ros/humble/setup.bash
            source /workspaces/soarm-car/install/setup.bash
            colcon --log-base /tmp/hm3d_regression/log build \\
                --base-paths ros2_ws/src/soarm_description ros2_ws/src/soarm_simulation \\
                --build-base /tmp/hm3d_regression/build --install-base /tmp/hm3d_regression/install --symlink-install
            source /tmp/hm3d_regression/install/setup.bash
            ros2 launch soarm_simulation simulation.launch.py world:=$world
        " >/dev/null

    ready=false
    for _ in {1..180}; do
        if docker exec "$container" bash -lc "source /opt/ros/humble/setup.bash; source /workspaces/soarm-car/install/setup.bash; source /tmp/hm3d_regression/install/setup.bash; ROS_DOMAIN_ID=$domain_id ros2 topic list" >"$temporary/topics" 2>/dev/null \
            && grep -qx /camera/image "$temporary/topics" \
            && grep -qx /camera/depth_image "$temporary/topics" \
            && grep -qx /mid360/points "$temporary/topics"; then
            ready=true
            break
        fi
        sleep 1
    done
    $ready || { docker logs "$container" >&2; exit 1; }
    sleep 4

    docker exec -i "$container" bash -lc "source /opt/ros/humble/setup.bash; source /workspaces/soarm-car/install/setup.bash; source /tmp/hm3d_regression/install/setup.bash; export ROS_DOMAIN_ID=$domain_id; timeout 40 python3 -" <<PY
import json, math, struct, time
from pathlib import Path
import numpy as np
import rclpy
from sensor_msgs.msg import Image, PointCloud2
latest = {}
rclpy.init()
node = rclpy.create_node('${world}_regression_probe')
for topic, message_type, key in [('/camera/image', Image, 'rgb'), ('/camera/depth_image', Image, 'depth'), ('/mid360/points', PointCloud2, 'cloud')]:
    node.create_subscription(message_type, topic, lambda message, key=key: latest.__setitem__(key, message), 10)
end = time.monotonic() + 35
while len(latest) < 3 and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
assert len(latest) == 3, 'sensor messages incomplete'
rgb, depth, cloud = (latest[key] for key in ('rgb', 'depth', 'cloud'))
image = np.frombuffer(bytes(rgb.data), np.uint8).reshape(rgb.height, rgb.step)[:, :rgb.width * 3].reshape(rgb.height, rgb.width, 3)
if rgb.encoding.lower() == 'bgr8':
    image = image[:, :, ::-1]
assert rgb.encoding.lower() in ('rgb8', 'bgr8')
Path('/tmp/${world}.ppm').write_bytes(f'P6\n{rgb.width} {rgb.height}\n255\n'.encode() + image.tobytes())
depths = np.frombuffer(bytes(depth.data), np.float32)
fields = {field.name: field.offset for field in cloud.fields}
finite_points = 0
for index in range(0, cloud.width * cloud.height, max(1, cloud.width * cloud.height // 5000)):
    row, column = divmod(index, cloud.width)
    offset = row * cloud.row_step + column * cloud.point_step
    point = [struct.unpack_from('<f', cloud.data, offset + fields[name])[0] for name in ('x', 'y', 'z')]
    finite_points += int(all(math.isfinite(value) for value in point))
summary = {
    'rgb_size': [rgb.width, rgb.height],
    'rgb_unique_colors': int(len(np.unique(image.reshape(-1, 3), axis=0))),
    'depth_finite_ratio': round(float(np.isfinite(depths).mean()), 4),
    'finite_lidar_samples': finite_points,
}
Path('/tmp/${world}.json').write_text(json.dumps(summary, sort_keys=True) + '\n')
assert summary['rgb_unique_colors'] > 1000
assert summary['depth_finite_ratio'] > 0.2
assert finite_points > 100
PY
    docker cp "$container:/tmp/$world.ppm" "$temporary/$world.ppm" >/dev/null
    docker cp "$container:/tmp/$world.json" "$temporary/$world.json" >/dev/null
    baseline="$output_dir/$world.ppm"
    if [[ "$mode" == record ]]; then
        cp "$temporary/$world.ppm" "$baseline"
    else
        [[ -f "$baseline" ]] || { echo "missing local baseline $baseline; run $0 record" >&2; exit 1; }
        python3 - "$baseline" "$temporary/$world.ppm" <<'PY'
import sys
import numpy as np
def pixels(path):
    raw = open(path, 'rb').read()
    header, data = raw.split(b'\n255\n', 1)
    return np.frombuffer(data, np.uint8).astype(np.float32), header
reference, reference_header = pixels(sys.argv[1])
candidate, candidate_header = pixels(sys.argv[2])
assert reference_header == candidate_header and reference.shape == candidate.shape
mae = float(np.abs(reference - candidate).mean())
print('pixel_mae=%.3f' % mae)
assert mae <= 25.0, 'visual regression MAE %.3f exceeds 25.0' % mae
PY
    fi
    printf '%s ' "$world"
    cat "$temporary/$world.json"
    ! docker logs "$container" 2>&1 | grep -Eqi 'invalid mesh|failed to load geometry|missing texture|segmentation fault'
    cleanup_world
    trap - EXIT
done

printf 'HM3D apartment regression %s passed; local screenshots: %s\n' "$mode" "$output_dir"
