#!/usr/bin/env bash
# Bounded, local-only research run. No source asset is copied or modified.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
asset_dir="${AWS_SMALL_HOUSE_DIR:-/home/gxxl/soarm-car-companion-validation/artifacts/baselines/aws_small_house_world}"
output="${AWS_KNOWNSELFPOSE_OUTPUT:-$root/artifacts/aws-knownselfpose-$(date +%Y%m%dT%H%M%S)}"
name="${SOARM_TEST_CONTAINER:-aws-knownselfpose-$RANDOM-$RANDOM}"
domain="${ROS_DOMAIN_ID:-181}"
partition="${IGN_PARTITION:-$name-ign}"
# The HM3D overlay is the locally pinned NVIDIA/Ogre2-capable renderer image.
image="${SOARM_TEST_IMAGE:-soarm-car-ros2-dev:hm3d}"
[[ ! -e "$output" ]] || { echo "refusing to overwrite output $output" >&2; exit 2; }
docker container inspect "$name" >/dev/null 2>&1 && { echo "refusing existing container $name" >&2; exit 2; }
[[ -d "$asset_dir/.git" ]] || { echo "missing readonly AWS source cache" >&2; exit 2; }
mkdir -p "$output"
python3 - "$root" "$asset_dir" "$output" <<'PY'
import hashlib, json, subprocess, sys
from pathlib import Path
root, asset, out = map(Path, sys.argv[1:])
lock = json.loads((root/'config/aws_small_house_companion.json').read_text())['asset']
manifest = json.loads((root/lock['content_manifest']).read_text())
assert manifest['commit'] == lock['commit'] == 'e2f90621f0fd1a0c496f4e7fcca06138808bc2ec'
assert subprocess.check_output(['git','-C',str(asset),'rev-parse','HEAD'],text=True).strip() == lock['commit']
assert not subprocess.check_output(['git','-C',str(asset),'status','--porcelain'],text=True).strip()
actual = [Path('LICENSE'), Path('worlds/small_house.world')]
actual += sorted(p.relative_to(asset) for p in (asset/'models').rglob('*') if p.is_file())
assert set(manifest['files']) == {str(p) for p in actual} and len(actual) == 370
for relative, detail in manifest['files'].items():
    data=(asset/relative).read_bytes(); assert len(data)==detail['bytes'] and hashlib.sha256(data).hexdigest()==detail['sha256'], relative
(out/'source_lock_verification.json').write_text(json.dumps({'commit':lock['commit'],'consumed_file_count':len(actual),'manifest_sha256':hashlib.sha256((root/lock['content_manifest']).read_bytes()).hexdigest(),'source_clean':True},indent=2)+'\n')
PY
container_id=""
cleanup() {
  [[ -z "$container_id" ]] || docker logs "$container_id" >"$output/container.log" 2>&1 || true
  [[ -z "$container_id" ]] || docker rm -f "$container_id" >/dev/null 2>&1 || true
  # Include outer logs and raw sensor/map files in a recoverable repository artifact.
  tar -C "$output" -czf "$output/knownselfpose_physicalsim_evidence.tar.gz" \
      --exclude=knownselfpose_physicalsim_evidence.tar.gz . 2>/dev/null || true
}
trap cleanup EXIT
container_id="$(docker run -d --name "$name" --gpus all --network none --device /dev/dri --group-add "${VIDEO_GID:-39}" --group-add "${RENDER_GID:-105}" --security-opt label=disable -e ROS_DOMAIN_ID="$domain" -e ROS_LOCALHOST_ONLY=1 -e IGN_PARTITION="$partition" -e IGN_IP=127.0.0.1 -e XDG_RUNTIME_DIR=/tmp/runtime-root -e DISPLAY="${DISPLAY:-:0}" -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute -e LD_LIBRARY_PATH= -e LIBGL_DRIVERS_PATH= -e __GLX_VENDOR_LIBRARY_NAME=nvidia -e __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/nvidia-egl.json -v "$root:/feature:ro" -v "$asset_dir:/asset:ro" -v "$output:/output" -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "${XAUTHORITY:?XAUTHORITY is required for local GLX}:/tmp/.host-xauthority:ro" -e XAUTHORITY=/tmp/.host-xauthority -w /feature --entrypoint bash "$image" -lc '
set -eo pipefail
mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root
printf "%s\\n" "{\\"file_format_version\\":\\"1.0.0\\",\\"ICD\\":{\\"library_path\\":\\"libEGL_nvidia.so.0\\"}}" > /tmp/nvidia-egl.json
source /opt/ros/humble/setup.bash; source /workspaces/soarm-car/install/setup.bash
colcon --log-base /tmp/knownpose/log build --base-paths /feature/ros2_ws/src/soarm_simulation /feature/ros2_ws/src/soarm_navigation --build-base /tmp/knownpose/build --install-base /tmp/knownpose/install --symlink-install
source /tmp/knownpose/install/setup.bash
cp /feature/ros2_ws/src/soarm_navigation/behavior_trees/aws_knownselfpose_navigate.xml /output/active_navigate_tree.xml
cp /feature/ros2_ws/src/soarm_navigation/behavior_trees/aws_knownselfpose_through_poses.xml /output/active_through_poses_tree.xml
python3 /feature/scripts/prepare_companion_home_world.py /asset/worlds/small_house.world /tmp/runtime.sdf
export IGN_GAZEBO_RESOURCE_PATH=/asset/models:/tmp/knownpose/install/soarm_simulation/share/soarm_simulation/models
export GZ_SIM_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH
ign gazebo -r -s /tmp/runtime.sdf >/output/gazebo.log 2>&1 &
bridge="/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist /gazebo/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry /camera/image@sensor_msgs/msg/Image[gz.msgs.Image /camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image /mid360/scan/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked /world/default/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
ros2 run ros_gz_bridge parameter_bridge $bridge >/output/bridge.log 2>&1 &
exec timeout 900 python3 /feature/scripts/aws_knownselfpose_pilot.py /output
')"
# The pilot is bounded internally; retain its recoverable output even on failure.
if ! timeout 930 docker wait "$container_id" >/dev/null; then
  echo "container wait timed out" >&2; exit 1
fi
status="$(docker inspect -f '{{.State.ExitCode}}' "$container_id")"
[[ "$status" == 0 ]] || { echo "pilot exited $status" >&2; exit "$status"; }
printf 'knownselfpose physicalsim research pilot complete: %s\n' "$output"
