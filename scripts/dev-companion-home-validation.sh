#!/usr/bin/env bash
set -euo pipefail

# Runs only renderer/sensor diagnostics. It never publishes /cmd_vel or evaluates navigation.
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
lock="$root/config/aws_small_house_companion.json"
asset_dir="${AWS_SMALL_HOUSE_DIR:-$root/artifacts/baselines/aws_small_house_world}"
image="${SOARM_TEST_IMAGE:-soarm-car-ros2-dev:hm3d}"
run_dir="${COMPANION_HOME_OUTPUT:?Set COMPANION_HOME_OUTPUT to a new output directory}"
container_name="${SOARM_TEST_CONTAINER:-soarm-companion-home-${RANDOM}-${RANDOM}}"
ros_domain_id="${ROS_DOMAIN_ID:-173}"
ign_partition="${IGN_PARTITION:-${container_name}-ign}"
host_display="${DISPLAY:?DISPLAY must name an existing local X server for Fortress Ogre2}"
host_xauthority="${XAUTHORITY:?XAUTHORITY must name local X credentials for Fortress Ogre2}"

[[ -f "$lock" && -d "$asset_dir/.git" ]] || { printf 'Acquire the pinned asset first with scripts/acquire-aws-small-house.sh\n' >&2; exit 2; }
[[ -S /tmp/.X11-unix/X0 && -f "$host_xauthority" ]] || { printf 'Required local X socket or Xauthority is unavailable\n' >&2; exit 2; }
[[ ! -e "$run_dir" ]] || { printf 'Refusing to overwrite output: %s\n' "$run_dir" >&2; exit 2; }
docker container inspect "$container_name" >/dev/null 2>&1 && { printf 'Refusing to use existing container name: %s\n' "$container_name" >&2; exit 2; }
mkdir -p "$run_dir"

container_id=""
cleanup() { [[ -z "$container_id" ]] || docker rm -f "$container_id" >/dev/null 2>&1 || true; }
trap cleanup EXIT

readarray -t lock_values < <(python3 - "$lock" <<'PY'
import json, sys
asset = json.load(open(sys.argv[1], encoding='utf-8'))['asset']
print(asset['commit']); print(asset['license_sha256']); print(asset['consumed_files']['worlds/small_house.world'])
PY
)
[[ "$(git -C "$asset_dir" rev-parse HEAD)" == "${lock_values[0]}" ]]
[[ "$(git -C "$asset_dir" status --porcelain)" == "" ]]
[[ "$(sha256sum "$asset_dir/LICENSE" | awk '{print $1}')" == "${lock_values[1]}" ]]
[[ "$(sha256sum "$asset_dir/worlds/small_house.world" | awk '{print $1}')" == "${lock_values[2]}" ]]
python3 - "$root" "$asset_dir" "$lock" <<'PY'
import hashlib, json, sys
from pathlib import Path
root, asset_dir, lock_path = map(Path, sys.argv[1:])
asset = json.loads(lock_path.read_text(encoding='utf-8'))['asset']
manifest_path = root / asset['content_manifest']
assert hashlib.sha256(manifest_path.read_bytes()).hexdigest() == asset['content_manifest_sha256']
manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
expected = manifest['files']
actual = [Path('LICENSE'), Path('worlds/small_house.world')]
actual.extend(sorted(path.relative_to(asset_dir) for path in (asset_dir / 'models').rglob('*') if path.is_file()))
assert set(expected) == {str(path) for path in actual}
for relative, details in expected.items():
    data = (asset_dir / relative).read_bytes()
    assert len(data) == details['bytes'] and hashlib.sha256(data).hexdigest() == details['sha256'], relative
PY
docker image inspect "$image" --format '{{.Id}} {{index .RepoDigests 0}}' >"$run_dir/image-identity.txt"

container_id="$(docker run -d --rm --name "$container_name" --gpus all --network none \
    --device /dev/dri --group-add "${VIDEO_GID:-39}" --group-add "${RENDER_GID:-105}" \
    --security-opt label=disable -e ROS_DOMAIN_ID="$ros_domain_id" -e ROS_LOCALHOST_ONLY=1 \
    -e IGN_PARTITION="$ign_partition" -e IGN_IP=127.0.0.1 \
    -e DISPLAY="$host_display" -e XAUTHORITY=/tmp/.host-xauthority \
    -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute -e LD_LIBRARY_PATH= -e LIBGL_DRIVERS_PATH= \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia -e __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/nvidia-egl.json \
    -e XDG_RUNTIME_DIR=/tmp/runtime-root \
    -v "$root:/feature:ro" -v "$asset_dir:/asset:ro" -v "$run_dir:/output" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "$host_xauthority:/tmp/.host-xauthority:ro" \
    -w /feature --entrypoint bash "$image" -lc '
        set -e
        mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root
        source /opt/ros/humble/setup.bash
        source /workspaces/soarm-car/install/setup.bash
        colcon --log-base /tmp/companion/log build --base-paths /feature/ros2_ws/src/soarm_simulation \
          --build-base /tmp/companion/build --install-base /tmp/companion/install --symlink-install
        source /tmp/companion/install/setup.bash
        python3 /feature/scripts/prepare_companion_home_world.py /asset/worlds/small_house.world /runtime.sdf
        export IGN_GAZEBO_RESOURCE_PATH=/asset/models:/tmp/companion/install/soarm_simulation/share/soarm_simulation/models
        export GZ_SIM_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH
        printf "%s\\n" "{\\\"file_format_version\\\":\\\"1.0.0\\\",\\\"ICD\\\":{\\\"library_path\\\":\\\"libEGL_nvidia.so.0\\\"}}" > /tmp/nvidia-egl.json
        { nvidia-smi -L; glxinfo -B; eglinfo -B || true; } > /output/renderer.txt 2>&1
        stdbuf -oL -eL ign gazebo -r -s /runtime.sdf > /output/gazebo.log 2>&1 &
        exec ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock /camera/image@sensor_msgs/msg/Image[gz.msgs.Image /camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo /camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image
    ')"

ros_setup='source /opt/ros/humble/setup.bash; source /workspaces/soarm-car/install/setup.bash; source /tmp/companion/install/setup.bash; export ROS_DOMAIN_ID='"$ros_domain_id"
ready=false
for _ in {1..120}; do
    if docker exec "$container_name" bash -lc "$ros_setup; ros2 topic list" >"$run_dir/topics.txt" 2>/dev/null \
        && grep -qx /camera/image "$run_dir/topics.txt" && grep -qx /camera/depth_image "$run_dir/topics.txt" && grep -qx /camera/camera_info "$run_dir/topics.txt"; then ready=true; break; fi
    sleep 1
done
$ready || { docker logs "$container_name" >"$run_dir/container.log" 2>&1 || true; exit 1; }

docker exec -i "$container_name" bash -lc "$ros_setup; timeout 80 python3 - /output /feature/config/aws_small_house_companion.json" <<'PY'
import json, math, struct, subprocess, sys, time, zlib
from datetime import datetime, timezone
from pathlib import Path
import numpy as np
import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image

output, lock_path = map(Path, sys.argv[1:])
viewpoints = json.loads(lock_path.read_text())['diagnostic_viewpoints']
state = {'clock_ns': 0, 'target_ns': None, 'rgb': None, 'depth': None, 'info': None, 'stale': {'rgb': 0, 'depth': 0}}
rclpy.init()
node = rclpy.create_node('companion_home_rgbd_probe')

def stamp_ns(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec

def accept_image(key, message):
    target = state['target_ns']
    if target is None:
        return
    if stamp_ns(message) <= target:
        state['stale'][key] += 1
        return
    state[key] = message

node.create_subscription(Clock, '/clock', lambda message: state.__setitem__('clock_ns', message.clock.sec * 10**9 + message.clock.nanosec), 20)
node.create_subscription(Image, '/camera/image', lambda message: accept_image('rgb', message), 20)
node.create_subscription(Image, '/camera/depth_image', lambda message: accept_image('depth', message), 20)
node.create_subscription(CameraInfo, '/camera/camera_info', lambda message: state.__setitem__('info', message), 20)

def spin_for(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)

def receive_after_pose():
    # Drain queued messages after pose acknowledgement before setting a simulation-time fence.
    state.update(target_ns=None, rgb=None, depth=None, info=None, stale={'rgb': 0, 'depth': 0})
    spin_for(.5)
    fence_ns = state['clock_ns']
    assert fence_ns > 0, 'no /clock observed after pose acknowledgement'
    state['target_ns'] = fence_ns + 500_000_000
    end = time.monotonic() + 15
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
        rgb, depth = state['rgb'], state['depth']
        if rgb is not None and depth is not None and state['info'] is not None:
            delta = abs(stamp_ns(rgb) - stamp_ns(depth))
            if delta <= 50_000_000:
                return rgb, depth, state['info'], fence_ns, state['target_ns'], delta, dict(state['stale'])
            state['rgb'] = state['depth'] = None
    raise AssertionError(f'no fresh synchronized RGB-D after fence {fence_ns}: stale={state["stale"]}')

def png(path, array):
    height, width, channels = array.shape
    color = 2 if channels == 3 else 0
    raw = b''.join(b'\0' + row.tobytes() for row in array)
    chunk = lambda tag, data: struct.pack('>I', len(data)) + tag + data + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff)
    path.write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, color, 0, 0, 0)) + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))

def rgb_array(message):
    channels = 3 if message.encoding in ('rgb8', 'bgr8') else 4
    array = np.frombuffer(bytes(message.data), np.uint8).reshape(message.height, message.width, channels)
    if message.encoding == 'bgr8': array = array[:, :, ::-1]
    return array[:, :, :3]

def depth_array(message):
    assert message.encoding in ('32FC1', '16UC1'), message.encoding
    dtype = np.float32 if message.encoding == '32FC1' else np.uint16
    values = np.frombuffer(bytes(message.data), dtype).reshape(message.height, message.width)
    return values if dtype is np.float32 else values.astype(np.float32) / 1000.0

records = []
for point in viewpoints:
    # Labelled diagnostic teleport only; no command velocity or navigation API is used.
    x, y, z, _, _, yaw = point['pose']
    request = ('name: "kuruma" position { x: %s y: %s z: %s } orientation { x: 0 y: 0 z: %s w: %s }' % (x, y, z, math.sin(yaw / 2), math.cos(yaw / 2)))
    response = subprocess.run(['ign', 'service', '-s', '/world/default/set_pose', '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req', request], text=True, capture_output=True)
    assert response.returncode == 0 and 'data: true' in response.stdout, response.stdout + response.stderr
    rgb, depth, info, fence_ns, target_ns, sync_delta_ns, stale = receive_after_pose()
    pixels, values = rgb_array(rgb), depth_array(depth)
    assert info.width == rgb.width == depth.width and info.height == rgb.height == depth.height
    assert info.k[0] > 0 and info.k[4] > 0, info.k
    finite = values[np.isfinite(values) & (values > 0)]
    assert finite.size > values.size * .2, (point['id'], finite.size, values.size)
    unique_colors = int(np.unique(pixels.reshape(-1, 3), axis=0).shape[0])
    assert unique_colors > 64, point['id']
    png(output / (point['id'] + '_rgb.png'), pixels)
    scaled = np.zeros_like(values, dtype=np.uint8)
    valid = np.isfinite(values) & (values > 0)
    scaled[valid] = np.clip((finite.max() - values[valid]) / max(finite.max() - finite.min(), .001) * 255, 0, 255).astype(np.uint8)
    png(output / (point['id'] + '_depth.png'), scaled[:, :, None])
    records.append({'id': point['id'], 'diagnostic_teleport': True, 'requested_pose_world': point['pose'], 'actual_pose_world': 'NOT VERIFIED', 'post_pose_clock_fence_ns': fence_ns, 'post_pose_accept_after_ns': target_ns, 'rgb_stamp_ns': stamp_ns(rgb), 'depth_stamp_ns': stamp_ns(depth), 'camera_info_stamp_ns': stamp_ns(info), 'camera_info_post_pose_fence': stamp_ns(info) > target_ns, 'camera_info_frame_matches_rgb': info.header.frame_id == rgb.header.frame_id, 'rgb_depth_sync_delta_ns': sync_delta_ns, 'stale_messages_discarded': stale, 'frames': [rgb.header.frame_id, depth.header.frame_id, info.header.frame_id], 'dimensions': [rgb.width, rgb.height], 'camera_info_fx_fy': [info.k[0], info.k[4]], 'rgb_unique_colors': unique_colors, 'depth_finite_positive_ratio': float(finite.size / values.size), 'depth_min_m': float(finite.min()), 'depth_max_m': float(finite.max())})
assert len({record['rgb_unique_colors'] for record in records}) > 1
(output / 'results.json').write_text(json.dumps({'capture_run_utc': datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z'), 'consumer_seam': 'Gazebo rgbd_camera -> ros_gz_bridge -> rclpy Image/CameraInfo subscriber', 'transport_isolation': {'docker_network': 'none', 'ros_localhost_only': True, 'ign_ip': '127.0.0.1', 'host_dds_reachability': 'DISABLED'}, 'viewpoints': records, 'map_correspondence': 'NOT VERIFIED; no occupancy map or PCD was consumed.', 'navigation_claim': 'NONE; poses are diagnostic teleports only.'}, indent=2) + '\n')
PY

docker exec "$container_name" bash -lc "IGN_PARTITION='$ign_partition' ign model --list" >"$run_dir/models.txt"
grep -Eq '^[[:space:]]*-[[:space:]]+kuruma$' "$run_dir/models.txt"
grep -Eq 'Bed_01_001|Sofa_01_001|KitchenTable_01_001' "$run_dir/models.txt"
docker logs "$container_name" >"$run_dir/container.log" 2>&1 || true
! grep -Eqi 'invalid mesh|failed to load geometry|missing texture|unable to find.*(mesh|texture)' "$run_dir/gazebo.log" "$run_dir/container.log"
grep -Eq 'direct rendering: Yes' "$run_dir/renderer.txt"
grep -Eq 'OpenGL renderer string: NVIDIA GeForce RTX 3070' "$run_dir/renderer.txt"
python3 - "$run_dir" "$lock" "$asset_dir" <<'PY'
import hashlib, json, subprocess, sys
from pathlib import Path
output, lock, asset = map(Path, sys.argv[1:])
result = json.loads((output / 'results.json').read_text())
result['source_lock'] = json.loads(lock.read_text())['asset']
result['asset_commit_observed'] = subprocess.check_output(['git', '-C', str(asset), 'rev-parse', 'HEAD'], text=True).strip()
result['image_identity'] = (output / 'image-identity.txt').read_text().strip()
result['review_image_sha256'] = {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(output.glob('*.png'))}
(output / 'evidence.json').write_text(json.dumps(result, indent=2) + '\n')
PY
printf 'Companion home RGB-D validation passed: %s\n' "$run_dir"
