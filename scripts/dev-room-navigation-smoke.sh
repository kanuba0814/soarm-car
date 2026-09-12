#!/usr/bin/env bash
# Real HTTP -> model-protocol fixture -> room executor -> FAST-LIO/Nav2 consumer.
# The local fixture is NOT a language model. No hardware or external model is used.
set -euo pipefail
cd "$(dirname "$0")/.."
[[ -s artifacts/fastlio_house_floorplan.pcd ]] || { echo 'Existing house PCD required.' >&2; exit 1; }
name="soarm-room-navigation-smoke-$$"
domain="${ROS_DOMAIN_ID:-197}"
partition="${IGN_PARTITION:-$name}"
gui_port="${SOARM_NAV_QA_GUI_PORT:-18194}"
export SOARM_NAV_QA_MODEL_PORT="${SOARM_NAV_QA_MODEL_PORT:-18195}"
logs="$(mktemp -d /tmp/soarm-room-navigation-smoke.XXXXXX)"
container=''; fixture_pid=''
cleanup() {
    if [[ -n "$container" ]]; then
        docker logs "$container" > "$logs/container.log" 2>&1 || true
        docker stop -t 10 "$container" >/dev/null 2>&1 || true
        docker rm "$container" >/dev/null 2>&1 || true
    fi
    if [[ -n "$fixture_pid" ]]; then
        kill "$fixture_pid" 2>/dev/null || true
        wait "$fixture_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT
printf 'EVIDENCE_DIR=%s\n' "$logs"
# Refuse occupied test ports instead of accidentally using an unrelated server.
python3 - "$gui_port" "$SOARM_NAV_QA_MODEL_PORT" <<'PY'
import socket, sys
for port in sys.argv[1:]:
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', int(port)))
PY
python3 ros2_ws/src/soarm_room_navigation/test/planner_fixture_server.py > "$logs/planner-fixture.log" 2>&1 &
fixture_pid=$!
container="$(docker compose run -d --name "$name" --no-deps \
    -e ROS_DOMAIN_ID="$domain" -e IGN_PARTITION="$partition" \
    -e SOARM_NAV_LLM_BASE_URL="http://127.0.0.1:$SOARM_NAV_QA_MODEL_PORT/v1" \
    -e SOARM_NAV_LLM_MODEL=protocol-fixture-only \
    -e SOARM_NAV_LLM_API_KEY=local-fixture-not-a-secret \
    ros2-dev bash -lc "ros2 launch soarm_room_navigation house_room_navigation.launch.py gui_port:=$gui_port rooms_path:=/tmp/qa-rooms.json task_journal:=/tmp/qa-task.json")"
source_ros='source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash'
inside() { docker exec "$container" timeout 10 bash -lc "$source_ros && ros2 $*"; }
ready=false
for _ in {1..120}; do
    nodes="$(inside 'node list' 2>/dev/null || true)"
    if [[ "$nodes" == *'/global_localization'* && "$nodes" == *'/room_navigation'* ]]; then ready=true; break; fi
    sleep 1
done
$ready
# Actual sensor-derived data must precede initial pose. Never wait for Nav2
# activation first: it depends on the map TF produced after initialization.
for topic in /Odometry /cloud_registered /global_map; do
    docker exec "$container" timeout 30 bash -lc "$source_ros && ros2 topic echo $topic --once --no-daemon" >/dev/null
done
inside "topic pub --times 6 -r 2 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'" > "$logs/initialpose.log"
ready=false
for _ in {1..90}; do
    planner="$(inside 'lifecycle get /planner_server' 2>/dev/null || true)"
    controller="$(inside 'lifecycle get /controller_server' 2>/dev/null || true)"
    navigator="$(inside 'lifecycle get /bt_navigator' 2>/dev/null || true)"
    if [[ "$planner" == *active* && "$controller" == *active* && "$navigator" == *active* ]]; then ready=true; break; fi
    sleep 1
done
$ready
inside 'param get /room_navigation use_sim_time' > "$logs/use_sim_time.txt"
grep -q True "$logs/use_sim_time.txt"
inside 'node info /room_navigation' > "$logs/room_navigation_info.txt"
! grep -q '/ground_truth/' "$logs/room_navigation_info.txt"
inside 'topic info /ground_truth/odom --verbose' > "$logs/truth_subscribers.txt"
! grep -Eq 'Node name: (global_localization|transform_fusion|localization_cmd_gate|room_navigation|controller_server|planner_server|bt_navigator)' "$logs/truth_subscribers.txt"
inside 'topic info /odom --verbose' > "$logs/odom_publishers.txt"
grep -q 'Publisher count: 1' "$logs/odom_publishers.txt"
grep -q 'Node name: transform_fusion' "$logs/odom_publishers.txt"
docker exec "$container" timeout 30 bash -lc "$source_ros && python3 -m pytest -q -p no:cacheprovider /workspaces/soarm-car/ros2_ws/src/soarm_navigation/test/test_localization_cmd_gate.py" > "$logs/gate-tests.log"
docker exec -i -e SOARM_NAV_QA_URL="http://127.0.0.1:$gui_port" "$container" timeout 180 bash -lc "$source_ros && python3 -" \
    < ros2_ws/src/soarm_room_navigation/test/consumer_probe.py > "$logs/http-consumer.json" 2> "$logs/http-consumer.stderr"
printf 'ROOM_NAVIGATION_SMOKE_PASSED evidence=%s\n' "$logs"
