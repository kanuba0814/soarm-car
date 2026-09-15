#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/dabai-dc1-common.sh"
usb=$(lsusb)
for pid in 0557 0657; do
    grep -qi "2bc5:$pid" <<< "$usb" || { echo "未检测到 DC1 USB 2bc5:$pid，请检查连接。" >&2; exit 2; }
done
if docker container inspect "$DC1_CONTAINER" >/dev/null 2>&1; then
    owner=$(docker inspect -f '{{index .Config.Labels "soarm.dc1.workspace"}}' "$DC1_CONTAINER")
    [[ "$owner" == "$ROOT_DIR" ]] || { echo "容器名称已被其他任务使用：$DC1_CONTAINER" >&2; exit 2; }
    if [[ $(docker inspect -f '{{.State.Running}}' "$DC1_CONTAINER") != true ]]; then
        docker rm "$DC1_CONTAINER" >/dev/null
    fi
fi
if ! docker container inspect "$DC1_CONTAINER" >/dev/null 2>&1; then
    docker run -d --name "$DC1_CONTAINER" --init \
        --label "soarm.dc1.workspace=$ROOT_DIR" \
        --network host --ipc host --device /dev/bus/usb:/dev/bus/usb \
        -e ROS_DOMAIN_ID="$DC1_ROS_DOMAIN_ID" \
        -v "$ROOT_DIR/scripts:/dc1:ro" \
        --entrypoint bash "$DC1_IMAGE" -lc \
        'source /opt/ros/humble/setup.bash
         source /opt/astra_ros2/setup.bash
         exec ros2 launch astra_camera dabai_dc1.launch.xml \
           uvc_product_id:=0x0557 enable_ir:=false enable_point_cloud:=false \
           depth_registration:=true depth_height:=400 tf_publish_rate:=0.0' >/dev/null
fi
report="$ROOT_DIR/artifacts/perception-dev/dabai-dc1/live-check.json"
if ! docker exec "$DC1_CONTAINER" bash -lc \
    'source /opt/ros/humble/setup.bash; python3 /dc1/dabai-dc1-check.py --seconds 10' > "$report"; then
    cat "$report"
    docker logs --tail 35 "$DC1_CONTAINER" >&2
    echo '相机检查未通过；请勿开始正式录制。' >&2
    exit 2
fi
python3 - "$report" <<'PYREPORT'
import json, sys
report = json.load(open(sys.argv[1]))
for warning in report.get('warnings', []):
    print('帧率提示（允许录制）：' + warning)
print('实测：RGB %.2f Hz，深度 %.2f Hz' % (
    report['rates_hz']['/camera/color/image_raw'],
    report['rates_hz']['/camera/depth/image_raw']))
PYREPORT
echo "DC1 已就绪，检查结果：$report"
