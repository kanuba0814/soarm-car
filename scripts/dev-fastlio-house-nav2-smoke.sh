#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
[[ -s artifacts/fastlio_house_floorplan.pcd ]] || ./scripts/dev-generate-house-pcd.sh
python3 - <<'PY'
from pathlib import Path
import yaml
config = yaml.safe_load(Path('vendor/localization/src/FAST_LIO_LOCALIZATION_ROS2/config/soarm_house_localization.yaml').read_text())
assert config['/**']['ros__parameters']['common'] == {'lid_topic': '/livox/lidar', 'imu_topic': '/livox/imu', 'time_sync_en': False, 'time_offset_lidar_to_imu': 0.0}
assert config['/**']['ros__parameters']['pcd_save']['pcd_save_en'] is False
assert config['/global_map_publisher']['ros__parameters']['map_file_path'].endswith('fastlio_house_floorplan.pcd')
assert config['/**']['ros__parameters']['publish']['odom_tf_en'] is False
nav = yaml.safe_load(Path('ros2_ws/src/soarm_navigation/config/house_nav2.yaml').read_text())
assert nav['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']['mid360']['topic'] == '/mid360/points'
PY

name="soarm-fastlio-house-$$"; domain="${ROS_DOMAIN_ID:-92}"; partition="${IGN_PARTITION:-$name}"
logs="$(mktemp -d /tmp/soarm-fastlio-house.XXXXXX)"
container="$(docker compose run -d --name "$name" -e ROS_DOMAIN_ID="$domain" -e IGN_PARTITION="$partition" ros2-dev bash -lc 'ros2 launch soarm_navigation house_fastlio_nav2.launch.py')"
cleanup() { docker logs "$container" >"$logs/container.log" 2>&1 || true; docker stop "$container" >/dev/null 2>&1 || true; }
trap cleanup EXIT
ros='source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && ros2'
inside() { docker exec "$container" bash -lc "$ros $*"; }
for _ in {1..180}; do
    nodes="$(inside 'node list' 2>/dev/null || true)"
    [[ "$nodes" == *'/global_localization'* && "$nodes" == *'/planner_server'* && "$nodes" != *amcl* ]] && break
    sleep 1
done
[[ "$(inside 'node list')" == *'/planner_server'* ]]
# No global transform can precede an accepted ICP correction.
if inside 'run tf2_ros tf2_echo map odom_combined' >/tmp/fastlio-preinit-tf 2>&1 & probe=$!; then sleep 2; kill "$probe" 2>/dev/null || true; wait "$probe" 2>/dev/null || true; ! grep -q 'Translation:' /tmp/fastlio-preinit-tf; fi
for topic in /global_map /cloud_registered /Odometry; do
    docker exec "$container" bash -lc "$ros topic echo $topic --once --no-daemon" >/dev/null
done
inside "topic pub --times 6 -r 2 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'" >/dev/null
for _ in {1..120}; do
    docker logs "$container" 2>&1 | grep -q 'Initial global localization succeeded.' && break
    sleep 1
done
docker logs "$container" 2>&1 | grep -q 'Initial global localization succeeded.'
# Consume an actual correction rather than relying on graph discovery timing.
docker exec "$container" bash -lc "$ros topic echo /map_to_odom --once --no-daemon" >"$logs/map_to_odom.yaml" & correction_probe=$!
sleep 10
kill "$correction_probe" 2>/dev/null || true
wait "$correction_probe" 2>/dev/null || true
grep -q 'frame_id: map' "$logs/map_to_odom.yaml"

docker exec -i "$container" bash -lc "source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && python3 -" >"$logs/metrics.json" <<'PY'
import json, math, subprocess, time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from rclpy.action import ActionClient

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
rclpy.init(); n=rclpy.create_node('fastlio_house_acceptance')
state={'odom':None, 'occupied':0, 'samples':[]}
def odom(msg):
    state['odom']=msg; state['samples'].append((msg.header.stamp.sec+msg.header.stamp.nanosec/1e9, msg.pose.pose.position.x, msg.pose.pose.position.y, yaw(msg.pose.pose.orientation), msg.twist.twist.linear.x, msg.twist.twist.angular.z))
def cost(msg): state['occupied']=max(state['occupied'], sum(cell >= 100 for cell in msg.data))
n.create_subscription(Odometry, '/odom', odom, 30); n.create_subscription(Costmap, '/local_costmap/costmap_raw', cost, 10)
c=ActionClient(n, NavigateToPose, '/navigate_to_pose'); assert c.wait_for_server(timeout_sec=45)
end=time.monotonic()+30
while state['odom'] is None and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.2)
assert state['odom'] is not None
start=state['odom']; g=NavigateToPose.Goal(); g.pose=PoseStamped(); g.pose.header.frame_id='map'; g.pose.pose.position.x=1.0; g.pose.pose.position.y=1.0; g.pose.pose.orientation.z=math.sin(math.pi/8); g.pose.pose.orientation.w=math.cos(math.pi/8)
f=c.send_goal_async(g)
rclpy.spin_until_future_complete(n,f,timeout_sec=20); h=f.result(); assert h and h.accepted
r=h.get_result_async(); rclpy.spin_until_future_complete(n,r,timeout_sec=120); assert r.result() and r.result().status==4
settle=time.monotonic()+1.5
while time.monotonic()<settle:rclpy.spin_once(n,timeout_sec=.1)
o=state['odom']
raw_truth=subprocess.check_output(['ign','topic','-e','-t','/world/house_floorplan/dynamic_pose/info','-n','1','--json-output'],text=True,timeout=10)
truth_message=json.loads(raw_truth)
truth=next(pose for pose in truth_message['pose'] if pose.get('name')=='senior_akm')
truth_position=truth.get('position',{})
truth_orientation=truth.get('orientation',{})
truth_x=truth_position.get('x',0.0); truth_y=truth_position.get('y',0.0)
class TruthQuaternion: pass
truth_q=TruthQuaternion()
truth_q.x=truth_orientation.get('x',0.0); truth_q.y=truth_orientation.get('y',0.0)
truth_q.z=truth_orientation.get('z',0.0); truth_q.w=truth_orientation.get('w',1.0)
error=math.hypot(o.pose.pose.position.x-1,o.pose.pose.position.y-1)
truth_error=math.hypot(o.pose.pose.position.x-truth_x,o.pose.pose.position.y-truth_y)
yaw_error=abs(math.atan2(math.sin(yaw(o.pose.pose.orientation)-yaw(truth_q)),math.cos(yaw(o.pose.pose.orientation)-yaw(truth_q))))
turn=max(abs(math.atan2(math.sin(s[3]-yaw(start.pose.pose.orientation)),math.cos(s[3]-yaw(start.pose.pose.orientation)))) for s in state['samples'])
lateral=max(abs(s[2]-start.pose.pose.position.y) for s in state['samples'])
active=[s for s in state['samples'] if abs(s[4])>.02 or abs(s[5])>.02]
metrics={'final_error_m':error,'truth_xy_error_m':truth_error,'truth_yaw_error_rad':yaw_error,'max_turn_rad':turn,'max_lateral_m':lateral,'occupied_cells':state['occupied'],'active_twist_samples':len(active),'final_twist':[o.twist.twist.linear.x,o.twist.twist.angular.z]}; print(json.dumps(metrics,sort_keys=True))
assert error<=.35 and truth_error<=.20 and yaw_error<=.10 and turn>=.35 and lateral>=.5 and state['occupied']>0 and active and abs(o.twist.twist.linear.x)<=.05 and abs(o.twist.twist.angular.z)<=.10
PY
cat "$logs/metrics.json"
printf 'FAST-LIO house Nav2 acceptance smoke passed.\n'
