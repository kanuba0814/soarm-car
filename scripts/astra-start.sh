#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
CONTAINER=soarm-astra-camera
IMAGE=${ASTRA_IMAGE:-soarm-car-astra-s-ros2-dev:latest}
ROS_DOMAIN=${ASTRA_ROS_DOMAIN_ID:-43}

if ! lsusb | grep -qi '2bc5:0402'; then
    echo "未检测到 Astra S（2bc5:0402），请检查 USB。" >&2
    exit 1
fi

mkdir -p "$ROOT_DIR/recordings"

gui_enabled=false
docker_options=(
    --name "$CONTAINER"
    --restart unless-stopped
    --network host
    --ipc host
    --device /dev/bus/usb:/dev/bus/usb
    -e ROS_DOMAIN_ID="$ROS_DOMAIN"
    -v "$ROOT_DIR/recordings:/recordings"
    -v "$ROOT_DIR/config/astra-rosbag-qos.yaml:/astra-rosbag-qos.yaml:ro"
    -v "$ROOT_DIR/config/astra-preview.rviz:/astra-preview.rviz:ro"
)

if [[ -n "${DISPLAY:-}" && -S /tmp/.X11-unix/X${DISPLAY#:} && -r "${XAUTHORITY:-}" && -d /dev/dri ]]; then
    gui_enabled=true
    docker_options+=(
        --device /dev/dri:/dev/dri
        --group-add "$(stat -c '%g' /dev/dri/card* | head -1)"
        --group-add "$(stat -c '%g' /dev/dri/renderD* | head -1)"
        -e DISPLAY="$DISPLAY"
        -e XAUTHORITY=/tmp/.host-xauthority
        -e XDG_RUNTIME_DIR=/tmp/runtime-root
        -e QT_X11_NO_MITSHM=1
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw
        -v "$XAUTHORITY:/tmp/.host-xauthority:ro"
    )
fi

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
    docker run -d \
        "${docker_options[@]}" \
        --entrypoint bash \
        "$IMAGE" -lc \
        'source /opt/ros/humble/setup.bash
         source /opt/astra_ros2/setup.bash
         exec ros2 launch astra_camera astra.launch.xml enable_ir:=false enable_color:=true enable_depth:=true enable_point_cloud:=true' \
        >/dev/null
fi

if ! docker exec "$CONTAINER" bash -lc \
    'source /opt/ros/humble/setup.bash
     source /opt/astra_ros2/setup.bash
     timeout 20 ros2 topic echo --once /camera/depth/image_raw sensor_msgs/msg/Image >/dev/null'; then
    echo "相机启动了，但没有收到深度帧。查看日志：docker logs soarm-astra-camera" >&2
    exit 1
fi

if [[ "$gui_enabled" == true ]] && ! docker exec "$CONTAINER" pgrep -x rviz2 >/dev/null 2>&1; then
    docker exec -d "$CONTAINER" bash -lc \
        'mkdir -p "$XDG_RUNTIME_DIR"
         chmod 700 "$XDG_RUNTIME_DIR"
         source /opt/ros/humble/setup.bash
         source /opt/astra_ros2/setup.bash
         exec rviz2 -d /astra-preview.rviz >/tmp/astra-rviz.log 2>&1'
    sleep 4
    if ! docker exec "$CONTAINER" pgrep -x rviz2 >/dev/null 2>&1; then
        docker exec "$CONTAINER" cat /tmp/astra-rviz.log >&2 || true
        echo "RViz 启动失败。" >&2
        exit 1
    fi
fi

if [[ "$gui_enabled" == true ]]; then
    echo "Astra S 与 RViz 实时预览已就绪。"
else
    echo "Astra S 已就绪；当前没有可用图形会话，未启动 RViz。"
fi
