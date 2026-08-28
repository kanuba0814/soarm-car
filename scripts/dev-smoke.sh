#!/usr/bin/env bash
set -euo pipefail
container="$(docker compose run -d --rm ros2-dev bash -lc 'ros2 launch soarm_fake simulation.launch.py rviz:=false foxglove:=false')"
cleanup() { docker stop "$container" >/dev/null 2>&1 || true; }
trap cleanup EXIT
ros_command='source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && ros2'
for _ in {1..30}; do
    if docker exec "$container" bash -lc "test -f /tmp/soarm_ros2/install/setup.bash && $ros_command action list | grep -qx /go_to_place_and_grasp"; then
        break
    fi
    sleep 1
done
docker exec "$container" bash -lc "test -f /tmp/soarm_ros2/install/setup.bash && $ros_command action list | grep -qx /go_to_place_and_grasp"
preview="$(docker exec "$container" bash -lc "$ros_command action send_goal /go_to_place_and_grasp soarm_interfaces/action/GoToPlaceAndGrasp '{request_id: demo-1, place_id: demo_table, object_id: demo_cube}' --feedback")"
printf '%s\n' "$preview"
grep -q "final_stage: visualization_preview" <<<"$preview"
grep -q "success: false" <<<"$preview"
grep -q "hardware execution is intentionally unavailable" <<<"$preview"
invalid="$(docker exec "$container" bash -lc "$ros_command action send_goal /go_to_place_and_grasp soarm_interfaces/action/GoToPlaceAndGrasp '{request_id: invalid-1, place_id: nowhere, object_id: demo_cube}'")"
printf '%s\n' "$invalid"
grep -q 'Goal was rejected.' <<<"$invalid"
