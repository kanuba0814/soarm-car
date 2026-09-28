#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
CONTAINER=soarm-astra-camera
SEQUENCE_ID=${1:-}

if [[ -z "$SEQUENCE_ID" && -n "${DISPLAY:-}" ]] && command -v zenity >/dev/null; then
    SEQUENCE_ID=$(zenity --list \
        --title="Astra S 场景录制" \
        --text="摆好场景后，双击一行即可开始录制" \
        --width=920 --height=430 \
        --column="标签" --column="场景" --column="怎么摆" \
        B01 "工作距离静态基线" "目标放在预计抓取距离；相机和物体完全静止" \
        B02 "距离阶梯" "同一目标依次放在 40/50/70/100 cm；每处静止约 6 秒" \
        B03 "深度困难材质" "同一距离摆放黑色、金属、透明和细物体" \
        B04 "正常目标变化" "目标依次放中央、左侧转 45°、右侧转 90°" \
        B05 "遮挡与杂乱" "依次保持无遮挡、20% 遮挡、50% 遮挡和杂物环境" \
        B06 "负样本" "先录空桌面，再只放干扰物；始终不放目标" \
        --print-column=1) || exit 0
elif [[ -z "$SEQUENCE_ID" ]]; then
    echo "B01  工作距离静态基线：目标放在预计抓取距离，全部静止"
    echo "B02  距离阶梯：40/50/70/100 cm，每处静止约 6 秒"
    echo "B03  深度困难材质：黑色、金属、透明、细物体"
    echo "B04  正常目标变化：中央、左转 45°、右转 90°"
    echo "B05  遮挡与杂乱：无遮挡、20%、50%、杂物"
    echo "B06  负样本：空桌面、只有干扰物，始终无目标"
    read -r -p "请输入标签（例如 B01）：" SEQUENCE_ID
fi

if [[ ! "$SEQUENCE_ID" =~ ^B0[1-6](-R[0-9]+)?$ ]]; then
    echo "标签无效。可用：B01 B02 B03 B04 B05 B06；复录可用 B01-R2。" >&2
    exit 1
fi

case "${SEQUENCE_ID%%-*}" in
    B01) SCENARIO="工作距离静态基线"; RECOMMENDED_SECONDS=15 ;;
    B02) SCENARIO="距离阶梯：40/50/70/100 cm"; RECOMMENDED_SECONDS=30 ;;
    B03) SCENARIO="深度困难材质：黑色/金属/透明/细物体"; RECOMMENDED_SECONDS=15 ;;
    B04) SCENARIO="正常目标：中央/左转45度/右转90度"; RECOMMENDED_SECONDS=30 ;;
    B05) SCENARIO="目标挑战：无遮挡/20%遮挡/50%遮挡/杂乱"; RECOMMENDED_SECONDS=30 ;;
    B06) SCENARIO="负样本：空桌面/只有干扰物"; RECOMMENDED_SECONDS=25 ;;
esac

BAG_NAME="${SEQUENCE_ID}_$(date +%Y%m%d-%H%M%S)"
CAPTURED_AT=$(date --iso-8601=seconds)

"$ROOT_DIR/scripts/astra-start.sh"

available_gb=$(df -BG --output=avail "$ROOT_DIR/recordings" | tail -1 | tr -dc '0-9')
if (( available_gb < 20 )); then
    echo "可用磁盘仅 ${available_gb} GB，不足以安全录制完整原始数据。" >&2
    exit 1
fi

echo "标签：$SEQUENCE_ID"
echo "场景：$SCENARIO"
echo "建议时长：约 ${RECOMMENDED_SECONDS} 秒"
echo "开始录制：recordings/$BAG_NAME（可用 ${available_gb} GB）"
echo "完成时按 Ctrl+C；请等待 rosbag 显示 Recording stopped。"

interrupted=false
stop_recording() {
    interrupted=true
    echo
    echo "正在安全停止并写完数据，请稍候……"
    docker exec "$CONTAINER" bash -lc \
        "pkill -INT -f '[r]os2 bag record.*$BAG_NAME' || true" 2>/dev/null || true
}
trap stop_recording INT TERM

record_log="$ROOT_DIR/recordings/.${BAG_NAME}.record.log"
docker exec -i "$CONTAINER" bash -lc \
    "source /opt/ros/humble/setup.bash
     source /opt/astra_ros2/setup.bash
     exec ros2 bag record \
       --max-cache-size 1073741824 \
       --max-bag-size 4294967296 \
       --qos-profile-overrides-path /astra-rosbag-qos.yaml \
       -o /recordings/$BAG_NAME \
       /camera/color/image_raw \
       /camera/color/camera_info \
       /camera/depth/image_raw \
       /camera/depth/camera_info \
       /camera/depth/points \
       /tf /tf_static" 2>&1 | tee "$record_log" &
recorder_client=$!

set +e
wait "$recorder_client"
record_status=$?
if kill -0 "$recorder_client" 2>/dev/null; then
    wait "$recorder_client"
    record_status=$?
fi
set -e
trap - INT TERM

if [[ "$record_status" -ne 0 && "$interrupted" != true ]]; then
    echo "录制异常停止（状态 $record_status）。" >&2
    exit "$record_status"
fi

# Store the scenario label and provenance beside the bag.
camera_serial=$(docker logs "$CONTAINER" 2>&1 | sed -n 's/.*Astra serial number: //p' | tail -1)
git_commit=$(git -C "$ROOT_DIR" rev-parse HEAD)
git_dirty=false
[[ -z "$(git -C "$ROOT_DIR" status --porcelain)" ]] || git_dirty=true
capture_manifest="$ROOT_DIR/recordings/.${BAG_NAME}.capture.yaml"
cat > "$capture_manifest" <<EOF
sequence_id: "$SEQUENCE_ID"
scenario: "$SCENARIO"
captured_at: "$CAPTURED_AT"
bag_name: "$BAG_NAME"
recording_profile: "full_rgb_depth_pointcloud"
camera:
    model: "Orbbec Astra S"
    usb_id: "2bc5:0402"
    serial: "$camera_serial"
source:
    git_commit: "$git_commit"
    git_dirty: $git_dirty
EOF

# Make captures easy to inspect/delete from the host even though rosbag runs in Docker.
if [[ -d "$ROOT_DIR/recordings/$BAG_NAME" ]]; then
    docker exec "$CONTAINER" mv \
        "/recordings/.${BAG_NAME}.record.log" "/recordings/$BAG_NAME/record.log"
    docker exec "$CONTAINER" mv \
        "/recordings/.${BAG_NAME}.capture.yaml" "/recordings/$BAG_NAME/capture.yaml"
fi
docker exec "$CONTAINER" chmod -R a+rX "/recordings/$BAG_NAME" 2>/dev/null || true

if grep -Eqi 'lost messages|dropped message|failed to write|disk full|no space left' \
    "$ROOT_DIR/recordings/$BAG_NAME/record.log"; then
    echo "警告：rosbag 日志报告了丢帧或写入错误，请不要使用这份数据。" >&2
    exit 2
fi

python3 "$ROOT_DIR/scripts/astra-verify-bag.py" "$ROOT_DIR/recordings/$BAG_NAME"
echo "录制完成且未检测到阻塞丢帧：recordings/$BAG_NAME"
