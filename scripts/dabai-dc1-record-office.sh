#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/dabai-dc1-common.sh"
NAME=${1:-office-$(date +%Y%m%d-%H%M%S)}
SECONDS_TO_RECORD=${2:-0}
[[ "$NAME" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || { echo 'Invalid bag name' >&2; exit 2; }
[[ "$SECONDS_TO_RECORD" =~ ^[0-9]+$ ]] || { echo 'Duration must be integer seconds (0 = Ctrl-C)' >&2; exit 2; }
[[ ! -e "$ROOT_DIR/recordings/$NAME" ]] || { echo "Refusing to overwrite recordings/$NAME" >&2; exit 2; }
"$ROOT_DIR/scripts/dabai-dc1-start.sh"
evidence="$ROOT_DIR/artifacts/perception-dev/dabai-dc1/$NAME"
mkdir -p "$evidence"
"$ROOT_DIR/scripts/dabai-dc1-hardware-check.sh" "$evidence" >/dev/null
cp "$ROOT_DIR/artifacts/perception-dev/dabai-dc1/live-check.json" "$evidence/"
docker logs "$DC1_CONTAINER" > "$evidence/driver.log" 2>&1
docker image inspect "$DC1_IMAGE" --format '{{.Id}}' > "$evidence/image-id.txt"
terminal=()
[[ -t 0 && -t 1 ]] && terminal=(-it)
echo "开始录制 $NAME；缓慢移动，回到起点。按 Ctrl-C 停止，然后等待自动校验完成。"
trap 'echo 正在结束录制并写入索引，请稍候……' INT
set +e
docker run --rm --init "${terminal[@]}" --network host --ipc host \
    -e ROS_DOMAIN_ID="$DC1_ROS_DOMAIN_ID" \
    -v "$ROOT_DIR/recordings:/recordings" -v "$ROOT_DIR/config:/dc1-config:ro" \
    --entrypoint bash "$DC1_IMAGE" -lc '
    source /opt/ros/humble/setup.bash
    command=(ros2 bag record --storage sqlite3 --max-bag-size 2147483648
      --qos-profile-overrides-path /dc1-config/dabai-dc1-rosbag-qos.yaml
      -o "/recordings/$1" /camera/color/image_raw /camera/color/camera_info
      /camera/depth/image_raw /camera/depth/camera_info /tf_static)
    if (( $2 > 0 )); then
      timeout --signal=INT --kill-after=20 "$2" "${command[@]}"
      status=$?
      [[ $status == 124 ]] && exit 0
      exit "$status"
    else
      exec "${command[@]}"
    fi' dc1-record "$NAME" "$SECONDS_TO_RECORD"
status=$?
set -e
trap - INT
if [[ $status != 0 && $status != 130 ]]; then
    echo "录制进程异常退出：$status" >&2
    exit "$status"
fi
"$ROOT_DIR/scripts/dabai-dc1-verify-bag.sh" "$NAME" | tee "$evidence/bag-check.json"
echo "录制及校验完成：$ROOT_DIR/recordings/$NAME"
