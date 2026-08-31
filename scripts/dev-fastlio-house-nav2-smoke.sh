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
localization = config['/global_localization']['ros__parameters']
assert localization['use_coarse_initialization'] is False
assert localization['max_correspondence_distance'] == 0.5
assert localization['max_correction_translation'] == 0.25
assert localization['max_correction_rotation'] == 0.15
assert localization['localization_rmse_th'] == 0.3
nav = yaml.safe_load(Path('ros2_ws/src/soarm_navigation/config/house_nav2.yaml').read_text())
assert nav['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']['mid360']['topic'] == '/mid360/points'
PY

name="soarm-fastlio-house-$$"; domain="${ROS_DOMAIN_ID:-92}"; partition="${IGN_PARTITION:-$name}"
logs="$(mktemp -d /tmp/soarm-fastlio-house.XXXXXX)"
container="$(docker compose run -d --name "$name" -e ROS_DOMAIN_ID="$domain" -e IGN_PARTITION="$partition" ros2-dev bash -lc 'ros2 launch soarm_navigation house_fastlio_nav2.launch.py')"
cleanup() {
    docker logs "$container" >"$logs/container.log" 2>&1 || true
    docker stop "$container" >/dev/null 2>&1 || true
    docker rm "$container" >/dev/null 2>&1 || true
}
trap cleanup EXIT
ros='source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && ros2'
inside() { docker exec "$container" bash -lc "$ros $*"; }
ready=false
for _ in {1..180}; do
    nodes="$(inside 'node list' 2>/dev/null || true)"
    if [[ "$nodes" == *'/global_localization'* && "$nodes" == *'/planner_server'* && "$nodes" != *amcl* ]]; then
        ready=true
        break
    fi
    sleep 1
done
$ready
# No global transform can precede an accepted ICP correction.
docker exec "$container" bash -lc \
    "source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && timeout 2 ros2 run tf2_ros tf2_echo map odom_combined" \
    >"$logs/preinit_tf.log" 2>&1 || [[ $? -eq 124 ]]
! grep -q 'Translation:' "$logs/preinit_tf.log"
wait_for_topic() {
    local topic=$1 expected=$2
    for _ in {1..90}; do
        [[ "$(inside "topic type $topic" 2>/dev/null || true)" == "$expected" ]] && return 0
        sleep 1
    done
    return 1
}
for topic_and_type in \
    '/global_map sensor_msgs/msg/PointCloud2' \
    '/cloud_registered sensor_msgs/msg/PointCloud2' \
    '/Odometry nav_msgs/msg/Odometry'; do
    read -r topic message_type <<<"$topic_and_type"
    wait_for_topic "$topic" "$message_type"
    docker exec "$container" timeout 30 bash -lc "$ros topic echo $topic $message_type --once --no-daemon" >/dev/null
done
inside "topic pub --times 6 -r 2 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'" >/dev/null
for _ in {1..120}; do
    docker logs "$container" >"$logs/readiness.log" 2>&1 || true
    grep -q 'Initial global localization succeeded.' "$logs/readiness.log" && break
    sleep 1
done
grep -q 'Initial global localization succeeded.' "$logs/readiness.log"
# Consume an actual correction rather than relying on graph discovery timing.
wait_for_topic /map_to_odom nav_msgs/msg/Odometry
docker exec "$container" timeout 30 bash -lc \
    "$ros topic echo /map_to_odom nav_msgs/msg/Odometry --once --no-daemon" \
    >"$logs/map_to_odom.yaml"
grep -q 'frame_id: map' "$logs/map_to_odom.yaml"
inside 'topic info /odom --verbose' >"$logs/odom_info.txt"
grep -q 'Publisher count: 1' "$logs/odom_info.txt"
grep -q 'Node name: transform_fusion' "$logs/odom_info.txt"
inside 'node info /kuruma_sim_adapter' >"$logs/sim_adapter_info.txt"
! grep -q '/gazebo/odom' "$logs/sim_adapter_info.txt"

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
state={'odom':None, 'localization':None, 'occupied':0, 'samples':[]}
def odom(msg):
    state['odom']=msg; state['samples'].append((msg.header.stamp.sec+msg.header.stamp.nanosec/1e9, msg.pose.pose.position.x, msg.pose.pose.position.y, yaw(msg.pose.pose.orientation), msg.twist.twist.linear.x, msg.twist.twist.angular.z))
def localization(msg): state['localization']=msg
def cost(msg): state['occupied']=max(state['occupied'], sum(cell >= 100 for cell in msg.data))
n.create_subscription(Odometry, '/odom', odom, 30); n.create_subscription(Odometry, '/localization', localization, 30); n.create_subscription(Costmap, '/local_costmap/costmap_raw', cost, 10)
c=ActionClient(n, NavigateToPose, '/navigate_to_pose'); assert c.wait_for_server(timeout_sec=45)
end=time.monotonic()+30
while (state['odom'] is None or state['localization'] is None) and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.2)
assert state['odom'] is not None and state['localization'] is not None
start=state['odom']

def fortress_pose():
    raw=subprocess.check_output(
        ['ign','topic','-e','-t','/world/house_floorplan/dynamic_pose/info','-n','1','--json-output'],
        text=True,timeout=10)
    truth=next(pose for pose in json.loads(raw)['pose'] if pose.get('name')=='kuruma')
    position=truth.get('position',{}); orientation=truth.get('orientation',{})
    class TruthQuaternion: pass
    q=TruthQuaternion(); q.x=orientation.get('x',0.0); q.y=orientation.get('y',0.0)
    q.z=orientation.get('z',0.0); q.w=orientation.get('w',1.0)
    return position.get('x',0.0),position.get('y',0.0),yaw(q)

goals=[
    (1.0,1.0,math.pi/4),
    (0.3,1.84,math.pi),
    (-1.7,1.84,math.pi),
    (0.3,1.84,0.0),
]
checkpoints=[]
for index,(goal_x,goal_y,goal_yaw) in enumerate(goals):
    g=NavigateToPose.Goal(); g.pose=PoseStamped(); g.pose.header.frame_id='map'
    g.pose.pose.position.x=goal_x; g.pose.pose.position.y=goal_y
    g.pose.pose.orientation.z=math.sin(goal_yaw/2); g.pose.pose.orientation.w=math.cos(goal_yaw/2)
    sent=c.send_goal_async(g); rclpy.spin_until_future_complete(n,sent,timeout_sec=20)
    handle=sent.result(); status=-1
    if handle and handle.accepted:
        result=handle.get_result_async(); rclpy.spin_until_future_complete(n,result,timeout_sec=120)
        if result.result() is not None: status=result.result().status
    settle=time.monotonic()+1.0
    while time.monotonic()<settle:rclpy.spin_once(n,timeout_sec=.1)
    o=state['localization']; truth_x,truth_y,truth_yaw=fortress_pose()
    checkpoint={
        'goal_index':index,
        'status':status,
        'goal_error_m':math.hypot(o.pose.pose.position.x-goal_x,o.pose.pose.position.y-goal_y),
        'truth_xy_error_m':math.hypot(o.pose.pose.position.x-truth_x,o.pose.pose.position.y-truth_y),
        'truth_yaw_error_rad':abs(math.atan2(math.sin(yaw(o.pose.pose.orientation)-truth_yaw),math.cos(yaw(o.pose.pose.orientation)-truth_yaw))),
    }
    checkpoints.append(checkpoint)
    if status != 4: break

o=state['localization']
turn=max(abs(math.atan2(math.sin(s[3]-yaw(start.pose.pose.orientation)),math.cos(s[3]-yaw(start.pose.pose.orientation)))) for s in state['samples'])
lateral=max(abs(s[2]-start.pose.pose.position.y) for s in state['samples'])
active=[s for s in state['samples'] if abs(s[4])>.02 or abs(s[5])>.02]
metrics={'checkpoints':checkpoints,'max_turn_rad':turn,'max_lateral_m':lateral,'occupied_cells':state['occupied'],'active_twist_samples':len(active),'final_twist':[o.twist.twist.linear.x,o.twist.twist.angular.z]}; print(json.dumps(metrics,sort_keys=True))
assert len(checkpoints)==len(goals)
assert all(point['status']==4 for point in checkpoints[:3])
assert checkpoints[-1]['status'] in (4,6)
assert all(point['goal_error_m']<=.35 for point in checkpoints[:3])
assert all(point['truth_xy_error_m']<=.20 and point['truth_yaw_error_rad']<=.10 for point in checkpoints)
assert turn>=.35 and lateral>=.5 and state['occupied']>0 and active
assert abs(o.twist.twist.linear.x)<=.05 and abs(o.twist.twist.angular.z)<=.10
PY
cat "$logs/metrics.json"
printf 'FAST-LIO house Nav2 acceptance smoke passed.\n'
