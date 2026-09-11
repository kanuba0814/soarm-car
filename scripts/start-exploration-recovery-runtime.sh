#!/usr/bin/env bash
set -euo pipefail

# Reuse the recovered navigation assets; never overwrite the historical run.
root="${NAV2_ASSET_ROOT:?Set NAV2_ASSET_ROOT to the audited recovered navigation workspace}"
name="${EXPLORATION_CONTAINER:?Set a unique EXPLORATION_CONTAINER}"
output="${EXPLORATION_RUNTIME_OUTPUT:?Set a new absolute evidence directory}"
image="sha256:1da9a18609c4664d7441d9f334c95e971f0795aa3815f6707ed761f2ce93d227"
domain="${ROS_DOMAIN_ID:-83}"
: "${DISPLAY:?DISPLAY is required}" "${XAUTHORITY:?XAUTHORITY is required}"
[[ -f "$XAUTHORITY" && -d "$root/ros2_ws/src" && "$output" = /* ]]
[[ "$domain" =~ ^[0-9]+$ ]] && (( domain < 100 ))
if docker container inspect "$name" >/dev/null 2>&1; then
    echo "Refusing existing container: $name" >&2
    exit 2
fi
if docker ps --format '{{.Names}}' | grep -Eq 'soarm-.*(runtime|demo|exploration)'; then
    echo 'Another simulation owner may be running; inspect before starting.' >&2
    exit 2
fi
docker image inspect "$image" --format '{{.Id}}' | grep -qx "$image"
mkdir "$output"
docker run -d --name "$name" --gpus all --network none --ipc private --shm-size 256m \
    --device /dev/dri/card0:/dev/dri/card0 --device /dev/dri/renderD129:/dev/dri/renderD129 \
    --group-add 105 --security-opt label=disable \
    -e ROS_DOMAIN_ID="$domain" -e ROS_LOCALHOST_ONLY=1 \
    -e IGN_PARTITION="${name}-${domain}" -e IGN_IP=127.0.0.1 \
    -e DISPLAY -e XAUTHORITY=/tmp/.host-xauthority \
    -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
    -e LD_LIBRARY_PATH= -e LIBGL_DRIVERS_PATH= -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/nvidia-egl.json -e XDG_RUNTIME_DIR=/tmp/runtime-root \
    -v "$root:/workspaces/soarm-car" -v "$output:/output" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "$XAUTHORITY:/tmp/.host-xauthority:ro" \
    -w /workspaces/soarm-car "$image" bash -lc '
        source /opt/ros/humble/setup.bash
        source /tmp/soarm_ros2/install/setup.bash
        mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root
        python3 -c '\''import json; json.dump({"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}},open("/tmp/nvidia-egl.json","w"))'\''
        exec ros2 launch soarm_navigation house_fastlio_nav2.launch.py rviz:=false use_native_livox_converter:=true throttle_odom_path:=false
    ' > "$output/container-id.txt"
docker inspect "$name" > "$output/docker-inspect.json"
printf 'Started %s; inspect build and sensor readiness before sending goals.\n' "$name"
