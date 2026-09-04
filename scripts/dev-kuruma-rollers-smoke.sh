#!/usr/bin/env bash
# Real Fortress/DART seam for the additive explicit-roller prototype.
set -euo pipefail
python3 scripts/check-kuruma-rollers.py
ros_domain_id="${ROS_DOMAIN_ID:-149}"
ign_partition="${IGN_PARTITION:-kuruma_rollers_smoke_$$}"
container="$(docker compose run -d --rm -e ROS_DOMAIN_ID="$ros_domain_id" -e IGN_PARTITION="$ign_partition" ros2-dev bash -lc 'ros2 launch soarm_simulation simulation.launch.py world:=kuruma_rollers rviz:=false')"
cleanup() { docker stop "$container" >/dev/null 2>&1 || true; }
trap cleanup EXIT
ros_env='source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash'
ros="$ros_env && ros2"
for _ in {1..120}; do
    if docker exec "$container" test -f /tmp/soarm_ros2/install/setup.bash; then break; fi
    sleep 1
done
docker exec "$container" test -f /tmp/soarm_ros2/install/setup.bash
for _ in {1..120}; do
    if docker exec "$container" bash -lc "output=\$($ros topic info /gazebo/odom 2>/dev/null) && [[ \$output == *'Publisher count: 1'* ]]"; then break; fi
    sleep 1
done
docker exec "$container" bash -lc "output=\$($ros topic info /gazebo/odom) && [[ \$output == *'Publisher count: 1'* ]]"
snapshot() { docker exec "$container" bash -lc "$ros_env && timeout 10 ros2 topic echo /gazebo/odom --once"; }
joints() { docker exec "$container" bash -lc "$ros_env && timeout 10 ros2 topic echo /joint_states --once"; }
for mode in x y yaw; do
    before="$(snapshot)"; before_joint="$(joints)"
    case "$mode" in
      x) cmd='{linear: {x: 0.20, y: 0.0}, angular: {z: 0.0}}' ;;
      y) cmd='{linear: {x: 0.0, y: 0.20}, angular: {z: 0.0}}' ;;
      yaw) cmd='{linear: {x: 0.0, y: 0.0}, angular: {z: 0.40}}' ;;
    esac
    docker exec "$container" bash -lc "$ros topic pub -r 20 --times 80 /cmd_vel geometry_msgs/msg/Twist '$cmd' >/dev/null"
    after="$(snapshot)"; after_joint="$(joints)"
    printf '%s\n---JOINTS-BEFORE---\n%s\n---BEFORE---\n%s\n---JOINTS-AFTER---\n%s\n---AFTER---\n%s\n' "$mode" "$before_joint" "$before" "$after_joint" "$after" > "/tmp/kuruma-rollers-$mode.yaml"
    docker exec "$container" bash -lc "$ros topic pub -r 20 --times 20 /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null"
done
zero_before="$(snapshot)"; sleep 10; zero_after="$(snapshot)"
printf '%s\n---AFTER---\n%s\n' "$zero_before" "$zero_after" > /tmp/kuruma-rollers-zero.yaml
# Keep sim alive for a total >=30 s after command exercise and inspect no non-finite state.
sleep 12
final="$(snapshot)"; printf '%s\n' "$final" > /tmp/kuruma-rollers-final.yaml
docker exec "$container" bash -lc 'timeout 10 ign topic -e -t /world/kuruma_rollers_lab/stats -n 1' > /tmp/kuruma-rollers-stats.txt
python3 - <<'PY'
import math, re, yaml
from pathlib import Path
def pose(text):
    x,y=map(float,re.search(r'position:\n      x: ([^\n]+)\n      y: ([^\n]+)',text).groups()); z,w=map(float,re.search(r'orientation:\n      x: [^\n]+\n      y: [^\n]+\n      z: ([^\n]+)\n      w: ([^\n]+)',text).groups()); return x,y,2*math.atan2(z,w)
def pair(path):
    a,b=Path(path).read_text().split('---AFTER---'); p,q=pose(a),pose(b); return q[0]-p[0],q[1]-p[1],q[2]-p[2]
def joint_positions(text, marker, end_marker):
    block = text.split(marker, 1)[1].split(end_marker, 1)[0]
    block = block[block.index('header:'):].split('\n---', 1)[0]
    message = yaml.safe_load(block)
    return dict(zip(message['name'], message['position']))

results={name:pair('/tmp/kuruma-rollers-'+name+'.yaml') for name in ('x','y','yaw')}; zero=pair('/tmp/kuruma-rollers-zero.yaml')
assert all(math.isfinite(v) for values in results.values() for v in values), results
assert abs(results['x'][0]) > abs(results['x'][1]) and abs(results['x'][0]) > .02, results
assert abs(results['y'][1]) > abs(results['y'][0]) and abs(results['y'][1]) > .02, results
assert abs(results['yaw'][2]) > .05, results
assert math.hypot(zero[0],zero[1]) < .005 and abs(zero[2]) < .01, zero
text=Path('/tmp/kuruma-rollers-y.yaml').read_text()
before_joints = joint_positions(text, '---JOINTS-BEFORE---', '---BEFORE---')
after_joints = joint_positions(text, '---JOINTS-AFTER---', '---AFTER---')
for wheel in ('front_left', 'front_right', 'rear_left', 'rear_right'):
    changes = [abs(after_joints[name] - value) for name, value in before_joints.items()
               if name.startswith(wheel + '_wheel_roller_') and name.endswith('_joint')]
    assert changes and max(changes) > 1e-4, (wheel, changes)
stats = Path('/tmp/kuruma-rollers-stats.txt').read_text()
rtf = float(re.search(r'real_time_factor: ([^\n]+)', stats).group(1))
assert math.isfinite(rtf) and rtf >= 0.8, rtf
print('DART explicit-roller physical smoke passed:', results, 'zero_drift=', zero, 'RTF=', rtf)
PY
