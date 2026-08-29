#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

ros_domain_id="${ROS_DOMAIN_ID:-90}"
ign_partition="${IGN_PARTITION:-soarm_fastlio_accuracy_$$}"
mkdir -p artifacts
gui="${FASTLIO_GUI:-false}"
gz_args='-r -s'
[[ "$gui" == true ]] && gz_args='-r'
container="$(docker compose run -d --rm \
    -e ROS_DOMAIN_ID="$ros_domain_id" \
    -e IGN_PARTITION="$ign_partition" \
    ros2-dev bash -lc "ros2 launch soarm_simulation simulation.launch.py world:=navigation_room gz_args:='$gz_args' & ros2 launch fast_lio_localization mapping.launch.py use_sim_time:=true config_file:=soarm_mid360_sim.yaml rviz:=$gui")"
cleanup() { docker stop "$container" >/dev/null 2>&1 || true; }
trap cleanup EXIT

ros='source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && ros2'
inside() { docker exec "$container" bash -lc "$ros $*"; }
bounded_echo() {
    local seconds=$1 topic=$2 output=$3
    docker exec "$container" bash -lc "$ros topic echo '$topic' nav_msgs/msg/Odometry --once --no-daemon > /tmp/soarm-bounded-echo 2>&1 & pid=\$!; for ((i=0; i<$seconds; i++)); do if ! kill -0 \$pid 2>/dev/null; then wait \$pid; status=\$?; cat /tmp/soarm-bounded-echo; exit \$status; fi; sleep 1; done; kill \$pid 2>/dev/null; wait \$pid 2>/dev/null; cat /tmp/soarm-bounded-echo; exit 124" >"$output"
}

ready=false
for _ in {1..180}; do
    if docker exec "$container" bash -lc "lidar=\$($ros topic info /livox/lidar 2>/dev/null); imu=\$($ros topic info /livox/imu 2>/dev/null); odom=\$($ros topic info /Odometry 2>/dev/null); [[ \$lidar == *'Publisher count: 1'* ]] && [[ \$imu == *'Publisher count: 1'* ]] && [[ \$odom =~ 'Publisher count: '[1-9] ]]" 2>/dev/null; then
        ready=true
        break
    fi
    sleep 1
done
$ready || { docker logs "$container" >&2; exit 1; }

capture() {
    local index=$1
    bounded_echo 30 /Odometry "/tmp/fastlio-accuracy-fast-${index}.yaml"
    bounded_echo 30 /gazebo/odom "/tmp/fastlio-accuracy-truth-${index}.yaml"
}

drive() {
    local label=$1 linear=$2 angular=$3 samples=$4 index=$5
    printf 'Segment %s: linear=%s m/s angular=%s rad/s duration=%.1f s\n' "$label" "$linear" "$angular" "$(awk "BEGIN { print $samples / 20 }")"
    inside "topic pub -r 20 --times $samples /cmd_vel geometry_msgs/msg/Twist '{linear: {x: $linear}, angular: {z: $angular}}'" >/dev/null
    inside "topic pub -r 20 --times 12 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'" >/dev/null
    sleep 2
    capture "$index"
}

# Let gravity initialization and the first local map settle before establishing the common origin.
sleep 8
capture 0
drive forward 0.12 0.00 60 1
drive left 0.12 0.20 80 2
drive right 0.12 -0.20 120 3
drive reverse -0.10 0.00 60 4
drive left_again 0.10 0.18 80 5

python3 - <<'PY'
import json
import math
from pathlib import Path
import yaml

labels = ['origin', 'forward', 'left', 'right', 'reverse', 'left_again']


def load(path):
    documents = yaml.safe_load_all(Path(path).read_text(encoding='utf-8'))
    return next(document for document in documents if isinstance(document, dict) and 'pose' in document)


def pose(message):
    position = message['pose']['pose']['position']
    orientation = message['pose']['pose']['orientation']
    yaw = math.atan2(
        2.0 * (orientation['w'] * orientation['z'] + orientation['x'] * orientation['y']),
        1.0 - 2.0 * (orientation['y'] ** 2 + orientation['z'] ** 2),
    )
    return position['x'], position['y'], position['z'], yaw


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def distance(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


fast = [pose(load(f'/tmp/fastlio-accuracy-fast-{index}.yaml')) for index in range(len(labels))]
truth = [pose(load(f'/tmp/fastlio-accuracy-truth-{index}.yaml')) for index in range(len(labels))]
alignment_yaw = truth[0][3] - fast[0][3]
cos_yaw = math.cos(alignment_yaw)
sin_yaw = math.sin(alignment_yaw)


def align(value):
    dx = value[0] - fast[0][0]
    dy = value[1] - fast[0][1]
    return (
        truth[0][0] + cos_yaw * dx - sin_yaw * dy,
        truth[0][1] + sin_yaw * dx + cos_yaw * dy,
        truth[0][2] + value[2] - fast[0][2],
        wrap(truth[0][3] + value[3] - fast[0][3]),
    )


aligned = [align(value) for value in fast]
checkpoints = []
segments = []
for index, label in enumerate(labels):
    position_error = distance(aligned[index], truth[index])
    yaw_error = abs(wrap(aligned[index][3] - truth[index][3]))
    checkpoints.append({
        'label': label,
        'position_error_m': position_error,
        'yaw_error_rad': yaw_error,
        'fastlio_aligned_xy': list(aligned[index][:2]),
        'gazebo_xy': list(truth[index][:2]),
    })
    if index:
        fast_distance = distance(fast[index - 1], fast[index])
        truth_distance = distance(truth[index - 1], truth[index])
        fast_yaw = wrap(fast[index][3] - fast[index - 1][3])
        truth_yaw = wrap(truth[index][3] - truth[index - 1][3])
        segments.append({
            'label': label,
            'fastlio_distance_m': fast_distance,
            'gazebo_distance_m': truth_distance,
            'distance_error_m': abs(fast_distance - truth_distance),
            'fastlio_yaw_delta_rad': fast_yaw,
            'gazebo_yaw_delta_rad': truth_yaw,
            'yaw_error_rad': abs(wrap(fast_yaw - truth_yaw)),
        })

fast_path = sum(segment['fastlio_distance_m'] for segment in segments)
truth_path = sum(segment['gazebo_distance_m'] for segment in segments)
summary = {
    'input': '/livox/lidar CustomMsg + /livox/imu',
    'trajectory': [segment['label'] for segment in segments],
    'fastlio_path_length_m': fast_path,
    'gazebo_path_length_m': truth_path,
    'path_length_error_m': abs(fast_path - truth_path),
    'path_length_error_percent': 100.0 * abs(fast_path - truth_path) / truth_path,
    'max_checkpoint_position_error_m': max(item['position_error_m'] for item in checkpoints),
    'max_checkpoint_yaw_error_rad': max(item['yaw_error_rad'] for item in checkpoints),
    'segments': segments,
    'checkpoints': checkpoints,
}
Path('artifacts/fastlio_odometry_accuracy.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
print(json.dumps(summary, indent=2))

assert truth_path > 1.0, summary
assert truth[2][3] - truth[1][3] > 0.15, summary
assert wrap(truth[3][3] - truth[2][3]) < -0.3, summary
assert summary['path_length_error_percent'] < 15.0, summary
assert summary['max_checkpoint_position_error_m'] < 0.20, summary
assert summary['max_checkpoint_yaw_error_rad'] < 0.20, summary
PY

printf '%s\n' 'FAST-LIO multi-direction odometry accuracy smoke passed.'
