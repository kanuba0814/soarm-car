#!/usr/bin/env bash
set -euo pipefail
map_path="artifacts/fastlio_navigation_room.pcd"
[[ -s "$map_path" ]] || { printf 'Missing %s; run dev-fastlio-navigation-room-smoke.sh first.\n' "$map_path" >&2; exit 1; }
python3 - <<'PY'
from pathlib import Path
import yaml
config = yaml.safe_load(Path('vendor/localization/src/FAST_LIO_LOCALIZATION_ROS2/config/soarm_mid360_localization.yaml').read_text())
assert config['/global_map_publisher']['ros__parameters']['map_file_path'] == '/workspaces/soarm-car/artifacts/fastlio_navigation_room.pcd'
params = config['/**']['ros__parameters']
assert params['common']['lid_topic'] == '/livox/lidar'
assert params['common']['imu_topic'] == '/livox/imu'
assert params['mapping']['extrinsic_est_en'] is False
PY
ros_domain_id="${ROS_DOMAIN_ID:-88}"
ign_partition="${IGN_PARTITION:-soarm_fastlio_localization_$$}"
container="$(docker compose run -d --rm -e ROS_DOMAIN_ID="$ros_domain_id" -e IGN_PARTITION="$ign_partition" ros2-dev bash -lc 'ros2 launch soarm_simulation simulation.launch.py world:=navigation_room ground_truth_map_tf:=false & ros2 launch fast_lio_localization velodyne_localization.launch.py use_sim_time:=true config_file:=soarm_mid360_localization.yaml rviz:=false')"
cleanup() { docker stop "$container" >/dev/null 2>&1 || true; }
trap cleanup EXIT
ros='source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && ros2'
inside() { docker exec "$container" bash -lc "$ros $*"; }
bounded_echo() {
    local seconds=$1 topic=$2 output=$3
    docker exec "$container" bash -lc "$ros topic echo '$topic' --once > /tmp/soarm-bounded-echo 2>&1 & pid=\$!; for ((i=0; i<$seconds; i++)); do if ! kill -0 \$pid 2>/dev/null; then wait \$pid; status=\$?; cat /tmp/soarm-bounded-echo; exit \$status; fi; sleep 1; done; kill \$pid 2>/dev/null; wait \$pid 2>/dev/null; cat /tmp/soarm-bounded-echo; exit 124" >"$output"
}
ready=false
for _ in {1..180}; do
    if docker exec "$container" bash -lc "nodes=\$($ros node list 2>/dev/null); topics=\$($ros topic list 2>/dev/null); [[ \$nodes == *'/global_localization'* ]] && [[ \$nodes == *'/global_map_publisher'* ]] && [[ \$topics == *'/global_map'* ]] && [[ \$topics == *'/cloud_registered'* ]]" 2>/dev/null; then
        ready=true
        break
    fi
    sleep 1
done
$ready || { docker logs "$container" >&2; exit 1; }
# Verify imports in the actual runtime image, then wait for the map and scan consumers.
docker exec "$container" bash -lc "$ros -h >/dev/null && python3 -c 'import open3d, tf_transformations; print(open3d.__version__)'"
for topic in /global_map /cloud_registered /Odometry; do
    bounded_echo 120 "$topic" /tmp/fastlio-localization-probe.yaml || {
        docker logs "$container" >&2
        exit 1
    }
done
# The map was built from the same simulated origin; seed ICP near identity.
inside "topic pub --times 3 -r 2 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}'" >/dev/null
for topic in /map_to_odom /localization /localization/pose_estimator/pose_with_covariance /localization/twist_estimator/twist_with_covariance; do
    bounded_echo 120 "$topic" "/tmp/fastlio-$(basename "$topic").yaml" || {
        docker logs "$container" >&2
        exit 1
    }
done
inside 'run tf2_ros tf2_echo map camera_init' >/tmp/fastlio-map-camera-tf.txt 2>&1 & tf_probe=$!
sleep 3
kill "$tf_probe" 2>/dev/null || true
wait "$tf_probe" 2>/dev/null || true
grep -q 'Translation:' /tmp/fastlio-map-camera-tf.txt
docker logs "$container" 2>&1 | grep -q 'Initial global localization succeeded.'
printf '%s\n' 'FAST-LIO global localization smoke passed.'
