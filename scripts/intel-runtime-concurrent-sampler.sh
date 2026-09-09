#!/usr/bin/env bash
# Concurrent host sampler for the existing archive-restored Nav2 route runner.
set -euo pipefail
out=${1:?output directory required}
mkdir -p "$out"
container=""
end=$((SECONDS + 1200))
while (( SECONDS < end )); do
    if [[ -z "$container" ]]; then
        container=$(docker ps --filter 'name=soarm-nav2-clean-direct_mppi' --format '{{.ID}}' | head -1 || true)
        [[ -n "$container" ]] || { sleep 1; continue; }
        printf '%s\n' "$container" > "$out/container-id.txt"
        docker inspect "$container" > "$out/container-inspect.json"
        docker exec "$container" bash -lc 'until test -f /tmp/soarm_ros2/install/setup.bash; do sleep 1; done; source /opt/ros/humble/setup.bash; source /tmp/soarm_ros2/install/setup.bash; timeout 300 python3 -' > "$out/ros-observer.jsonl" 2>"$out/ros-observer.stderr" <<'PY' &
import json, time
import rclpy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header
from soarm_interfaces.msg import LocalizationHealth

rclpy.init(); node = rclpy.create_node('intel_runtime_scalar_observer')
state = {'clock': None}
def stamp(message):
    header = getattr(message, 'header', None)
    if header is None: return None
    value = header.stamp
    return value.sec + value.nanosec / 1e9
def emit(kind, message):
    row = {'receipt_monotonic': time.monotonic(), 'kind': kind, 'clock': state['clock'], 'stamp': stamp(message)}
    if kind == 'health':
        row.update({name: getattr(message, name) for name in ('state', 'map_id', 'consecutive_rejections', 'fitness', 'rmse')})
    print(json.dumps(row, allow_nan=True), flush=True)
def clock(message):
    state['clock'] = message.clock.sec + message.clock.nanosec / 1e9
node.create_subscription(Clock, '/clock', clock, 30)
node.create_subscription(LocalizationHealth, '/localization/health', lambda m: emit('health', m), 30)
for topic in ('/mid360/points', '/livox/lidar', '/gazebo/imu', '/gazebo/livox_imu', '/Odometry', '/cloud_registered', '/localization'):
    # Generic subscriptions are intentionally omitted: ros2 topic echo below records scalar headers/types without payload dumps.
    pass
while rclpy.ok(): rclpy.spin_once(node, timeout_sec=.1)
PY
        docker exec "$container" bash -lc 'source /opt/ros/humble/setup.bash; source /tmp/soarm_ros2/install/setup.bash; for topic in /mid360/points /livox/lidar /gazebo/imu /gazebo/livox_imu /Odometry /cloud_registered /localization; do echo "=== $topic"; ros2 topic type "$topic" || true; timeout 10 ros2 topic echo "$topic" --once --field header --no-daemon || true; done; echo "=== map_to_odom"; timeout 8 ros2 run tf2_ros tf2_echo map odom_combined || true; echo "=== odom_base"; timeout 8 ros2 run tf2_ros tf2_echo odom_combined base_footprint || true' > "$out/ros-scalar-seams.txt" 2>&1 || true
        docker exec "$container" bash -lc 'ps -eo pid,ppid,comm,args; echo ---DRI_FDS---; for p in $(pgrep -f "ign gazebo|gzserver|ruby /usr/bin/ign" || true); do echo PID=$p; ls -l /proc/$p/fd 2>/dev/null | grep /dev/dri || true; grep -E "libGL|libEGL|dri/|iris" /proc/$p/maps 2>/dev/null | head -30 || true; done' > "$out/renderer-process.txt" 2>&1 || true
    fi
    if docker inspect "$container" >/dev/null 2>&1; then
        printf '%s ' "$(date -u +%FT%TZ)" >> "$out/host-samples.log"
        docker stats --no-stream --format 'cpu={{.CPUPerc}} mem={{.MemUsage}} pids={{.PIDs}}' "$container" >> "$out/host-samples.log" 2>&1 || true
        docker exec "$container" bash -lc 'ps -eo pid,psr,pcpu,pmem,stat,nlwp,rss,comm --sort=-pcpu | head -35; cat /sys/fs/cgroup/cpu.stat 2>/dev/null || true; grep Cpus_allowed_list /proc/1/status' >> "$out/process-samples.log" 2>&1 || true
    else
        break
    fi
    sleep 1
done
