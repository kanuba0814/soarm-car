#!/usr/bin/env bash
set -euo pipefail

# Starts one isolated, interactive Fortress instance. Its lifetime is the Gazebo GUI.
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
asset_dir="${AWS_SMALL_HOUSE_DIR:-$root/artifacts/baselines/aws_small_house_world}"
image="${SOARM_DEMO_IMAGE:-soarm-car-ros2-dev:hm3d}"
container_name="${SOARM_DEMO_CONTAINER:-soarm-companion-home-demo}"
ros_domain_id="${ROS_DOMAIN_ID:-174}"
ign_partition="${IGN_PARTITION:-${container_name}-ign}"
run_dir="${SOARM_DEMO_OUTPUT:-}"
host_display="${DISPLAY:?DISPLAY must name the local X server}"
host_xauthority="${XAUTHORITY:?XAUTHORITY must name local X credentials}"

[[ -d "$asset_dir/.git" ]] || { printf 'Missing pinned AWS house asset: %s\n' "$asset_dir" >&2; exit 2; }
[[ -S /tmp/.X11-unix/X0 && -f "$host_xauthority" ]] || { printf 'Local X socket or Xauthority is unavailable\n' >&2; exit 2; }
docker container inspect "$container_name" >/dev/null 2>&1 && { printf 'Demo container already exists: %s\nUse scripts/companion-home-demo-control.sh stop-demo first.\n' "$container_name" >&2; exit 2; }

if [[ -z "$run_dir" ]]; then
    run_dir="$(mktemp -d "/tmp/${container_name}.XXXXXX")"
else
    [[ ! -e "$run_dir" ]] || { printf 'Refusing to overwrite output: %s\n' "$run_dir" >&2; exit 2; }
    mkdir -p "$run_dir"
fi
printf '%s\n' "$image" >"$run_dir/image.txt"

container_id="$(docker run -d --name "$container_name" --gpus all --network none --device /dev/dri \
    --group-add "${VIDEO_GID:-39}" --group-add "${RENDER_GID:-105}" --security-opt label=disable \
    -e ROS_DOMAIN_ID="$ros_domain_id" -e ROS_LOCALHOST_ONLY=1 -e IGN_PARTITION="$ign_partition" -e IGN_IP=127.0.0.1 \
    -e DISPLAY="$host_display" -e XAUTHORITY=/tmp/.host-xauthority -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
    -e LD_LIBRARY_PATH= -e LIBGL_DRIVERS_PATH= -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/nvidia-egl.json -e XDG_RUNTIME_DIR=/tmp/runtime-root \
    -v "$root:/feature:ro" -v "$asset_dir:/asset:ro" -v "$run_dir:/output" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "$host_xauthority:/tmp/.host-xauthority:ro" \
    -w /feature --entrypoint bash "$image" -lc '
set -eo pipefail
mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root
source /opt/ros/humble/setup.bash
source /workspaces/soarm-car/install/setup.bash
colcon --log-base /tmp/companion-demo/log build --base-paths /feature/ros2_ws/src/soarm_simulation --build-base /tmp/companion-demo/build --install-base /tmp/companion-demo/install --symlink-install
source /tmp/companion-demo/install/setup.bash
python3 /feature/scripts/prepare_companion_home_world.py /asset/worlds/small_house.world /runtime.sdf
export IGN_GAZEBO_RESOURCE_PATH=/asset/models:/tmp/companion-demo/install/soarm_simulation/share/soarm_simulation/models
export GZ_SIM_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH
printf "%s\\n" "{\\"file_format_version\\":\\"1.0.0\\",\\"ICD\\":{\\"library_path\\":\\"libEGL_nvidia.so.0\\"}}" > /tmp/nvidia-egl.json
ros2 run ros_gz_bridge parameter_bridge \
  "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist" \
  "/gazebo/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
  "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
  "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image" > /output/bridge.log 2>&1 &
bridge_pid=$!
cleanup() { kill "$bridge_pid" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
ign gazebo -r /runtime.sdf > /output/gazebo.log 2>&1 &
gazebo_pid=$!
for _ in $(seq 1 60); do
  ros2 topic list 2>/dev/null | grep -qx /camera/image && break
  sleep 1
done
# A separate native preview window makes the robot camera visible during the demo.
ros2 run rqt_image_view rqt_image_view /camera/image > /output/camera-preview.log 2>&1 &
wait "$gazebo_pid"
')"

ready=false
for _ in $(seq 1 90); do
    if docker exec "$container_name" bash -lc 'source /opt/ros/humble/setup.bash; source /tmp/companion-demo/install/setup.bash; ros2 topic list' >"$run_dir/topics.txt" 2>/dev/null \
        && grep -qx /camera/image "$run_dir/topics.txt" && grep -qx /gazebo/odom "$run_dir/topics.txt"; then
        ready=true
        break
    fi
    sleep 1
done
if ! $ready; then
    docker logs "$container_name" >"$run_dir/container.log" 2>&1 || true
    printf 'Demo did not become ready; logs: %s\n' "$run_dir" >&2
    exit 1
fi

printf 'Companion home demo is running.\ncontainer=%s\ncontainer_id=%s\noutput=%s\nROS_DOMAIN_ID=%s\nIGN_PARTITION=%s\n' \
    "$container_name" "$container_id" "$run_dir" "$ros_domain_id" "$ign_partition"
