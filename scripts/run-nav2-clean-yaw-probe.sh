#!/usr/bin/env bash
# Run the bounded pure-yaw drivetrain calibration through the localization safety gate.
set -euo pipefail

repo="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo"
run_id="nav2-clean-yaw-probe-$(date -u +%Y%m%dT%H%M%SZ)"
out="artifacts/$run_id"
mkdir -p "$out"
container="soarm-nav2-clean-yaw-$$"
image="${NAV2_CLEAN_IMAGE:-soarm-car-ros2-dev}"
domain="${ROS_DOMAIN_ID:-223}"
partition="${IGN_PARTITION:-soarm_nav2_clean_yaw_probe}"

cleanup() {
    docker logs "$container" >"$out/container.log" 2>&1 || true
    docker stop -t 15 "$container" >/dev/null 2>&1 || true
    docker rm "$container" >/dev/null 2>&1 || true
}
trap cleanup EXIT

docker run -d --name "$container" --network host --ipc host --security-opt label=disable \
    --device /dev/dri --group-add "${VIDEO_GID:-39}" --group-add "${RENDER_GID:-105}" \
    -v "$repo:/workspaces/soarm-car" \
    -e ROS_DOMAIN_ID="$domain" -e IGN_PARTITION="$partition" \
    -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 -e MKL_NUM_THREADS=1 -e NUMEXPR_NUM_THREADS=1 \
    "$image" bash -lc \
    "ros2 launch soarm_navigation house_fastlio_nav2.launch.py nav2_params_file:=/workspaces/soarm-car/ros2_ws/src/soarm_navigation/config/house_nav2_mppi.yaml" \
    >"$out/container-id.txt"
ros='source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && ros2'
ready=false
for _ in {1..240}; do
    nodes="$(docker exec "$container" bash -lc "$ros node list" 2>/dev/null || true)"
    if [[ "$nodes" == *'/global_localization'* && "$nodes" == *'/localization_cmd_gate'* && "$nodes" == *'/controller_manager'* ]]; then
        ready=true
        break
    fi
    sleep 1
done
[[ "$ready" == true ]]
metrics="/workspaces/soarm-car/$out/metrics.json"
docker exec "$container" timeout --signal=INT --kill-after=20 240 bash -lc \
    "source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && python3 /workspaces/soarm-car/scripts/nav2-clean-yaw-probe.py '$metrics'" \
    >"$out/consumer.log" 2>&1
cat "$out/metrics.json"
printf 'Run artifacts: %s\n' "$out"
