#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
source scripts/astra-usb-device.sh
export ASTRA_USB_DEVICE="$(astra_usb_device)"
ros_domain_id="${ROS_DOMAIN_ID:-$((100 + RANDOM % 100))}"
container="$(docker compose -f compose.yaml -f compose.astra.yaml run -d \
    -e ROS_DOMAIN_ID="$ros_domain_id" -e ROS_LOCALHOST_ONLY=1 ros2-dev \
    ros2 launch soarm_astra hardware.launch.py)"
cleanup() { docker rm -f "$container" >/dev/null 2>&1 || true; }
trap cleanup EXIT
setup='source /opt/ros/humble/setup.bash && source /opt/astra_ros2/setup.bash && source /tmp/soarm_ros2/install/setup.bash'
for _ in {1..120}; do
    if [[ "$(docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null || true)" != true ]]; then
        docker logs "$container" >&2 || true
        echo 'Astra driver container exited before publishing depth images.' >&2
        exit 1
    fi
    if docker exec "$container" bash -lc "test -f /tmp/soarm_ros2/install/setup.bash && $setup && nodes=\$(ros2 node list 2>/dev/null) && [[ \$nodes == *'/camera/camera'* ]]"; then
        break
    fi
    sleep 1
done

docker exec "$container" bash -lc "$setup && nodes=\$(ros2 node list) && [[ \$nodes == *'/camera/camera'* ]]"
docker exec "$container" bash -lc "$setup && python3 /workspaces/soarm-car/scripts/astra-contract-check.py --timeout 25"
printf '%s\n' "Physical Astra S contract passed through $ASTRA_USB_DEVICE."
