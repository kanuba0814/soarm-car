#!/usr/bin/env bash
# Bounded real Nav2 action test against the saved widened house scene.
set -euo pipefail

repo="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo"
run_id="nav2-clean-direct-mppi-$(date -u +%Y%m%dT%H%M%SZ)"
out="artifacts/$run_id"
mkdir -p "$out"
image="${NAV2_CLEAN_IMAGE:-soarm-car-ros2-dev}"
domain_base="${ROS_DOMAIN_ID:-217}"
partition_base="${IGN_PARTITION:-soarm_nav2_clean_comparison}"

[[ -s artifacts/house_widened_reference.pcd ]]
[[ -s artifacts/rooms-complete-reference.json ]]
python3 - "$out" <<'PY'
import hashlib
import json
import pathlib
import sys
import yaml

root = pathlib.Path('.')
out = pathlib.Path(sys.argv[1])
paths = {
    'pcd': root / 'artifacts/house_widened_reference.pcd',
    'rooms': root / 'artifacts/rooms-complete-reference.json',
    'world_sdf': root / 'ros2_ws/src/soarm_simulation/worlds/house_floorplan_ros2_control.sdf',
    'model_sdf': root / 'ros2_ws/src/soarm_simulation/models/house_floorplan/model.sdf',
    'occupancy': root / 'ros2_ws/src/soarm_simulation/maps/house_floorplan.pgm',
}

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

rooms = json.loads(paths['rooms'].read_text())
route_ids = ('room-1', 'room-2', 'room-3', 'mian')
selected = [next(room for room in rooms['rooms'] if room['id'] == room_id) for room_id in route_ids]
assert [room['id'] for room in selected] == list(route_ids)
assert all(room['require_yaw'] is False for room in selected if room['id'].startswith('room-'))
configs = {
    'direct_mppi': root / 'ros2_ws/src/soarm_navigation/config/house_nav2_mppi.yaml',
    'rotation_shim': root / 'ros2_ws/src/soarm_navigation/config/house_nav2_mppi_shim.yaml',
}
parsed = {name: yaml.safe_load(path.read_text()) for name, path in configs.items()}
direct = parsed['direct_mppi']['controller_server']['ros__parameters']['FollowPath']
shim = parsed['rotation_shim']['controller_server']['ros__parameters']['FollowPath']
assert direct['plugin'] == 'nav2_mppi_controller::MPPIController'
assert shim['plugin'] == 'nav2_rotation_shim_controller::RotationShimController'
assert shim['primary_controller'] == direct['plugin']
direct_without_plugin = {key: value for key, value in direct.items() if key != 'plugin'}
shim_mppi = {key: value for key, value in shim.items() if key not in {
    'plugin', 'primary_controller', 'angular_dist_threshold', 'angular_disengage_threshold',
    'forward_sampling_distance', 'rotate_to_heading_angular_vel', 'max_angular_accel',
    'simulate_ahead_time', 'rotate_to_goal_heading'}}
assert direct_without_plugin == shim_mppi
for config in parsed.values():
    params = config['controller_server']['ros__parameters']
    assert params['goal_checker_plugins'] == ['goal_checker', 'position_only_goal_checker']
    assert params['position_only_goal_checker']['yaw_goal_tolerance'] == 3.141592653589793
out.joinpath('snapshot.json').write_text(json.dumps({
    'source_hashes': {name: digest(path) for name, path in paths.items()},
    'saved_route': selected,
    'configs': {name: {'path': str(path), 'sha256': digest(path)} for name, path in configs.items()},
    'comparison': 'Only FollowPath plugin/wrapper differs. Position-only goal checking is identical.',
}, indent=2, sort_keys=True) + '\n')
PY
diff -u ros2_ws/src/soarm_navigation/config/house_nav2_mppi.yaml \
    ros2_ws/src/soarm_navigation/config/house_nav2_mppi_shim.yaml >"$out/direct-vs-shim.diff" || true

cleanup_container() {
    local container=$1 case_dir=$2
    docker logs "$container" >"$case_dir/container.log" 2>&1 || true
    docker stop -t 15 "$container" >/dev/null 2>&1 || true
    docker rm "$container" >/dev/null 2>&1 || true
}

run_case() {
    local case_name=$1 domain=$2 partition=$3 params=$4
    local case_dir="$out/$case_name" container="soarm-nav2-clean-${case_name}-$$"
    local metrics_container="/workspaces/soarm-car/$case_dir/metrics.json"
    mkdir -p "$case_dir"
    docker run -d --name "$container" --network host --ipc host --security-opt label=disable \
        --device /dev/dri --group-add "${VIDEO_GID:-39}" --group-add "${RENDER_GID:-105}" \
        -v "$repo:/workspaces/soarm-car" \
        -e ROS_DOMAIN_ID="$domain" -e IGN_PARTITION="$partition" \
        -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 -e MKL_NUM_THREADS=1 -e NUMEXPR_NUM_THREADS=1 \
        "$image" bash -lc "ros2 launch soarm_navigation house_fastlio_nav2.launch.py nav2_params_file:=$params" \
        >"$case_dir/container-id.txt"
    local stop_needed=true
    trap 'if [[ "$stop_needed" == true ]]; then cleanup_container "$container" "$case_dir"; fi' RETURN
    local ros='source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && ros2'
    local ready=false nodes
    for _ in {1..240}; do
        nodes="$(docker exec "$container" bash -lc "$ros node list" 2>/dev/null || true)"
        if [[ "$nodes" == *'/global_localization'* && "$nodes" == *'/planner_server'* && "$nodes" == *'/bt_navigator'* ]]; then
            ready=true
            break
        fi
        sleep 1
    done
    if [[ "$ready" != true ]]; then
        cleanup_container "$container" "$case_dir"
        stop_needed=false
        trap - RETURN
        return 1
    fi
    local consumer_status=0
    docker exec -e NAV2_CLEAN_ROUTE="${NAV2_CLEAN_ROUTE:-all}" "$container" \
        timeout --signal=INT --kill-after=30 900 bash -lc \
        "source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && python3 /workspaces/soarm-car/scripts/nav2-clean-route-consumer.py '$metrics_container'" \
        >"$case_dir/consumer.log" 2>&1 || consumer_status=$?
    cleanup_container "$container" "$case_dir"
    stop_needed=false
    trap - RETURN
    return "$consumer_status"
}

case_status=0
selected_case="${NAV2_CLEAN_CASE:-direct_mppi}"
case "$selected_case" in
    direct_mppi)
        run_case direct_mppi "$domain_base" "${partition_base}_direct" \
            "/workspaces/soarm-car/ros2_ws/src/soarm_navigation/config/house_nav2_mppi.yaml" || case_status=$?
        ;;
    rotation_shim)
        run_case rotation_shim "$domain_base" "${partition_base}_shim" \
            "/workspaces/soarm-car/ros2_ws/src/soarm_navigation/config/house_nav2_mppi_shim.yaml" || case_status=$?
        ;;
    both)
        run_case direct_mppi "$domain_base" "${partition_base}_direct" \
            "/workspaces/soarm-car/ros2_ws/src/soarm_navigation/config/house_nav2_mppi.yaml" || case_status=$?
        run_case rotation_shim "$((domain_base + 1))" "${partition_base}_shim" \
            "/workspaces/soarm-car/ros2_ws/src/soarm_navigation/config/house_nav2_mppi_shim.yaml" || case_status=$?
        ;;
    *)
        printf 'Unknown NAV2_CLEAN_CASE: %s\n' "$selected_case" >&2
        exit 64
        ;;
esac
python3 - "$out" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
report = {'run_dir': str(root), 'cases': {}}
for name in ('direct_mppi', 'rotation_shim'):
    metrics = root / name / 'metrics.json'
    if metrics.exists():
        data = json.loads(metrics.read_text())
        report['cases'][name] = {
            'complete': data.get('complete', False),
            'passed': data.get('passed', False),
            'error': data.get('error'),
            'classification': data.get('classification'),
            'route': data.get('route', []),
        }
root.joinpath('summary.json').write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
print(json.dumps(report, indent=2, sort_keys=True))
PY
printf 'Run artifacts: %s\n' "$out"
exit "$case_status"
