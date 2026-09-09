#!/usr/bin/env bash
set -euo pipefail

# Runs bounded renderer/sensor diagnostic capture only. It never publishes /cmd_vel or evaluates navigation.
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
lock="$root/config/aws_small_house_companion.json"
capture_config="${COMPANION_CAPTURE_CONFIG:-$root/config/companion_metric_pilot.json}"
runtime_calibration_panel="${COMPANION_RUNTIME_CALIBRATION_PANEL:-0}"
evaluator_segmentation="${COMPANION_EVALUATOR_SEGMENTATION:-0}"
segmentation_smoke="${COMPANION_SEGMENTATION_OCCLUSION_SMOKE:-0}"
asset_dir="${AWS_SMALL_HOUSE_DIR:-$root/artifacts/baselines/aws_small_house_world}"
image="${SOARM_TEST_IMAGE:-soarm-car-ros2-dev:hm3d}"
run_dir="${COMPANION_HOME_OUTPUT:?Set COMPANION_HOME_OUTPUT to a new output directory}"
container_name="${SOARM_TEST_CONTAINER:-soarm-companion-home-${RANDOM}-${RANDOM}}"
ros_domain_id="${ROS_DOMAIN_ID:-173}"
ign_partition="${IGN_PARTITION:-${container_name}-ign}"
host_display="${DISPLAY:?DISPLAY must name an existing local X server for Fortress Ogre2}"
host_xauthority="${XAUTHORITY:?XAUTHORITY must name local X credentials for Fortress Ogre2}"

[[ -f "$lock" && -f "$capture_config" && -d "$asset_dir/.git" ]] || { printf 'Acquire the pinned asset/config first with scripts/acquire-aws-small-house.sh\n' >&2; exit 2; }
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
asset = json.loads(lock_path.read_text())['asset']; manifest_path = root / asset['content_manifest']
assert hashlib.sha256(manifest_path.read_bytes()).hexdigest() == asset['content_manifest_sha256']
manifest = json.loads(manifest_path.read_text())['files']
actual = [Path('LICENSE'), Path('worlds/small_house.world')]
actual.extend(sorted(path.relative_to(asset_dir) for path in (asset_dir / 'models').rglob('*') if path.is_file()))
assert set(manifest) == {str(path) for path in actual}
for relative, details in manifest.items():
    data = (asset_dir / relative).read_bytes(); assert len(data) == details['bytes'] and hashlib.sha256(data).hexdigest() == details['sha256'], relative
PY
docker image inspect "$image" --format '{{.Id}} {{index .RepoDigests 0}}' >"$run_dir/image-identity.txt"

container_id="$(docker run -d --rm --name "$container_name" --gpus all --network none --device /dev/dri --group-add "${VIDEO_GID:-39}" --group-add "${RENDER_GID:-105}" --security-opt label=disable -e ROS_DOMAIN_ID="$ros_domain_id" -e ROS_LOCALHOST_ONLY=1 -e IGN_PARTITION="$ign_partition" -e IGN_IP=127.0.0.1 -e DISPLAY="$host_display" -e XAUTHORITY=/tmp/.host-xauthority -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute -e LD_LIBRARY_PATH= -e LIBGL_DRIVERS_PATH= -e __GLX_VENDOR_LIBRARY_NAME=nvidia -e __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/nvidia-egl.json -e XDG_RUNTIME_DIR=/tmp/runtime-root -v "$root:/feature:ro" -v "$asset_dir:/asset:ro" -v "$run_dir:/output" -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "$host_xauthority:/tmp/.host-xauthority:ro" -w /feature --entrypoint bash "$image" -lc '
set -e; mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root
source /opt/ros/humble/setup.bash; source /workspaces/soarm-car/install/setup.bash
colcon --log-base /tmp/companion/log build --base-paths /feature/ros2_ws/src/soarm_simulation --build-base /tmp/companion/build --install-base /tmp/companion/install --symlink-install
source /tmp/companion/install/setup.bash
runtime_options=""
if [ "'"$runtime_calibration_panel"'" = 1 ]; then runtime_options="$runtime_options --metric-calibration-panel"; fi
if [ "'"$evaluator_segmentation"'" = 1 ]; then runtime_options="$runtime_options --evaluator-segmentation"; fi
if [ "'"$segmentation_smoke"'" = 1 ]; then runtime_options="$runtime_options --segmentation-occlusion-smoke"; fi
python3 /feature/scripts/prepare_companion_home_world.py /asset/worlds/small_house.world /runtime.sdf $runtime_options
export IGN_GAZEBO_RESOURCE_PATH=/tmp/companion-evaluator-models:/asset/models:/tmp/companion/install/soarm_simulation/share/soarm_simulation/models; export GZ_SIM_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH
printf "%s\\n" "{\\"file_format_version\\":\\"1.0.0\\",\\"ICD\\":{\\"library_path\\":\\"libEGL_nvidia.so.0\\"}}" > /tmp/nvidia-egl.json
{ nvidia-smi -L; glxinfo -B; eglinfo -B || true; } > /output/renderer.txt 2>&1
stdbuf -oL -eL ign gazebo -r -s /runtime.sdf > /output/gazebo.log 2>&1 &
bridge_topics="/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock /camera/image@sensor_msgs/msg/Image[gz.msgs.Image /camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo /camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image /world/default/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
if [ "'"$evaluator_segmentation"'" = 1 ]; then bridge_topics="$bridge_topics /eval_segmentation/labels_map@sensor_msgs/msg/Image[gz.msgs.Image /eval_segmentation/colored_map@sensor_msgs/msg/Image[gz.msgs.Image"; fi
exec ros2 run ros_gz_bridge parameter_bridge $bridge_topics
')"
ros_setup='source /opt/ros/humble/setup.bash; source /workspaces/soarm-car/install/setup.bash; source /tmp/companion/install/setup.bash; export ROS_DOMAIN_ID='"$ros_domain_id"
ready=false
for _ in {1..120}; do
    if docker exec "$container_name" bash -lc "$ros_setup; ros2 topic list" >"$run_dir/topics.txt" 2>/dev/null && grep -qx /camera/image "$run_dir/topics.txt" && grep -qx /camera/depth_image "$run_dir/topics.txt" && grep -qx /camera/camera_info "$run_dir/topics.txt" && grep -qx /world/default/dynamic_pose/info "$run_dir/topics.txt"; then ready=true; break; fi
    sleep 1
done
$ready || { docker logs "$container_name" >"$run_dir/container.log" 2>&1 || true; exit 1; }
docker exec "$container_name" bash -lc "$ros_setup; ros2 topic type /world/default/dynamic_pose/info; ros2 topic type /camera/image; ros2 topic type /camera/depth_image; ros2 topic type /camera/camera_info" >"$run_dir/topic-types.txt"
capture_config_in_container="/feature/${capture_config#$root/}"
docker exec -i "$container_name" bash -lc "$ros_setup; COMPANION_EVALUATOR_CAPTURE='$evaluator_segmentation' timeout 900 python3 /feature/scripts/companion_capture.py /output '$capture_config_in_container'"
docker exec "$container_name" bash -lc "IGN_PARTITION='$ign_partition' ign model --list" >"$run_dir/models.txt"
docker logs "$container_name" >"$run_dir/container.log" 2>&1 || true
! grep -Eqi 'invalid mesh|failed to load geometry|missing texture|unable to find.*(mesh|texture)' "$run_dir/gazebo.log" "$run_dir/container.log"
grep -Eq 'direct rendering: Yes' "$run_dir/renderer.txt"
python3 - "$run_dir" "$lock" "$asset_dir" <<'PY'
import hashlib, json, subprocess, sys
from pathlib import Path
output, lock, asset = map(Path, sys.argv[1:]); capture = json.loads((output / 'capture.json').read_text())
capture['source_lock'] = json.loads(lock.read_text())['asset']; capture['asset_commit_observed'] = subprocess.check_output(['git', '-C', str(asset), 'rev-parse', 'HEAD'], text=True).strip(); capture['image_identity'] = (output / 'image-identity.txt').read_text().strip()
capture['files_sha256'] = {str(path.relative_to(output)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(output.rglob('*')) if path.is_file() and path.name not in ('evidence.json',)}
(output / 'evidence.json').write_text(json.dumps(capture, indent=2) + '\n')
PY
printf 'Companion RGB-D observed-pose pilot passed: %s\n' "$run_dir"
