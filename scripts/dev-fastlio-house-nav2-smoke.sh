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
assert localization['planar_correction_gate'] is True
assert localization['max_correction_z'] == 0.5
assert localization['max_correction_roll_pitch'] == 0.15
assert localization['localization_rmse_th'] == 0.3
assert localization['quality_rejection_dead_reckoning'] is True
assert localization['max_unverified_translation'] == 5.0
assert localization['max_unverified_rotation'] == 3.0
assert localization['max_unverified_seconds'] == 40.0
nav = yaml.safe_load(Path('ros2_ws/src/soarm_navigation/config/house_nav2_mppi.yaml').read_text())
controller = nav['controller_server']['ros__parameters']['FollowPath']
assert controller['plugin'] == 'nav2_rotation_shim_controller::RotationShimController'
assert controller['primary_controller'] == 'nav2_mppi_controller::MPPIController'
assert controller['motion_model'] == 'Omni'
assert controller['vx_min'] == -0.25 and controller['vx_max'] == 0.25
assert controller['vy_max'] == 0.25 and controller['wz_max'] == 1.0
controller_server = nav['controller_server']['ros__parameters']
assert controller_server['progress_checker']['movement_time_allowance'] == 20.0
assert controller_server['goal_checker']['stateful'] is False
expected_footprint = '[[-0.30, -0.30], [-0.30, 0.30], [0.30, 0.30], [0.30, -0.30]]'
assert nav['global_costmap']['global_costmap']['ros__parameters']['footprint'] == expected_footprint
local_costmap = nav['local_costmap']['local_costmap']['ros__parameters']
assert local_costmap['footprint'] == expected_footprint
assert local_costmap['plugins'] == ['voxel_layer', 'inflation_layer']
assert local_costmap['voxel_layer']['mid360']['topic'] == '/mid360/points'
assert local_costmap['voxel_layer']['mid360']['obstacle_min_range'] == 0.20
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
controllers_ready=false
for _ in {1..90}; do
    controllers="$(inside 'control list_controllers' 2>/dev/null || true)"
    if grep -q 'joint_state_broadcaster.*active' <<<"$controllers" && grep -q 'mecanum_drive_controller.*active' <<<"$controllers"; then
        controllers_ready=true
        break
    fi
    sleep 1
done
$controllers_ready
for topic_and_type in \
    '/ground_truth/odom nav_msgs/msg/Odometry' \
    '/wheel/odom_raw nav_msgs/msg/Odometry' \
    '/imu/data_raw sensor_msgs/msg/Imu' \
    '/livox/imu sensor_msgs/msg/Imu' \
    '/livox/lidar livox_ros_driver2/msg/CustomMsg' \
    '/mid360/points sensor_msgs/msg/PointCloud2' \
    '/camera/image sensor_msgs/msg/Image' \
    '/camera/camera_info sensor_msgs/msg/CameraInfo' \
    '/camera/depth_image sensor_msgs/msg/Image'; do
    read -r topic message_type <<<"$topic_and_type"
    wait_for_topic "$topic" "$message_type"
done
for diagnostic_topic in /ground_truth/odom /wheel/odom_raw; do
    inside "topic info $diagnostic_topic --verbose" >"$logs/${diagnostic_topic//\//_}_info.txt"
    grep -q 'Publisher count: 1' "$logs/${diagnostic_topic//\//_}_info.txt"
done
inside 'topic info /ground_truth/odom --verbose' >"$logs/ground_truth_odom_info.txt"
! grep -Eq 'Node name: (global_localization|transform_fusion|controller_server|planner_server|bt_navigator)' "$logs/ground_truth_odom_info.txt"
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
inside 'topic info /tf --verbose' >"$logs/tf_info.txt"
grep -q 'Node name: transform_fusion' "$logs/tf_info.txt"
docker exec "$container" bash -lc \
    "source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && timeout 5 ros2 run tf2_ros tf2_echo odom_combined base_footprint" \
    >"$logs/odom_base_tf.log" 2>&1 || [[ $? -eq 124 ]]
grep -q 'Translation:' "$logs/odom_base_tf.log"
docker exec "$container" bash -lc \
    "source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && timeout 5 ros2 run tf2_ros tf2_echo map odom_combined" \
    >"$logs/map_odom_tf.log" 2>&1 || [[ $? -eq 124 ]]
grep -q 'Translation:' "$logs/map_odom_tf.log"
inside 'node info /kuruma_sim_adapter' >"$logs/sim_adapter_info.txt"
! grep -q '/gazebo/odom' "$logs/sim_adapter_info.txt"
nav_active=false
for _ in {1..120}; do
    planner_state="$(inside 'lifecycle get /planner_server' 2>/dev/null || true)"
    controller_state="$(inside 'lifecycle get /controller_server' 2>/dev/null || true)"
    navigator_state="$(inside 'lifecycle get /bt_navigator' 2>/dev/null || true)"
    if [[ "$planner_state" == *active* && "$controller_state" == *active* && "$navigator_state" == *active* ]]; then
        nav_active=true
        break
    fi
    sleep 1
done
$nav_active

docker exec -i "$container" bash -lc "source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash && python3 -" >"$logs/metrics.json" <<'PY'
import json, math, subprocess, time
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
rclpy.init(); n=rclpy.create_node('fastlio_house_acceptance')
state={'odom':None, 'localization':None, 'truth':None, 'clock':None, 'occupied':0, 'samples':[], 'commands':[]}
def odom(msg):
    state['odom']=msg; state['samples'].append((msg.header.stamp.sec+msg.header.stamp.nanosec/1e9, msg.pose.pose.position.x, msg.pose.pose.position.y, yaw(msg.pose.pose.orientation), msg.twist.twist.linear.x, msg.twist.twist.angular.z))
def localization(msg): state['localization']=msg
def truth(msg): state['truth']=msg
def command(msg): state['commands'].append((time.monotonic(),msg.linear.x,msg.linear.y,msg.angular.z))
def clock(msg): state['clock']=msg.clock.sec+msg.clock.nanosec/1e9
def cost(msg): state['occupied']=max(state['occupied'], sum(cell >= 100 for cell in msg.data))
n.create_subscription(Odometry, '/odom', odom, 30); n.create_subscription(Odometry, '/localization', localization, 30)
n.create_subscription(Odometry, '/ground_truth/odom', truth, 30); n.create_subscription(Twist, '/cmd_vel', command, 30)
n.create_subscription(Clock, '/clock', clock, 30); n.create_subscription(Costmap, '/local_costmap/costmap_raw', cost, 10)
c=ActionClient(n, NavigateToPose, '/navigate_to_pose'); assert c.wait_for_server(timeout_sec=45)
end=time.monotonic()+30
while (state['odom'] is None or state['localization'] is None or state['clock'] is None) and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.2)
assert state['odom'] is not None and state['localization'] is not None and state['clock'] is not None
start=state['odom']; start_sim=state['clock']; start_wall=time.monotonic()

def fortress_pose():
    raw=subprocess.check_output(
        ['ign','topic','-e','-t','/world/house_floorplan_ros2_control/dynamic_pose/info','-n','1','--json-output'],
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
command_tail=state['commands'][-240:]
rtf=(state['clock']-start_sim)/(time.monotonic()-start_wall)
metrics={'checkpoints':checkpoints,'max_turn_rad':turn,'max_lateral_m':lateral,'occupied_cells':state['occupied'],'active_twist_samples':len(active),'final_twist':[o.twist.twist.linear.x,o.twist.twist.angular.z], 'rtf':rtf, 'command_tail':command_tail[::max(1,len(command_tail)//40)]}; print(json.dumps(metrics,sort_keys=True))
assert len(checkpoints)==len(goals)
assert all(point['status']==4 for point in checkpoints)
assert all(point['goal_error_m']<=.35 for point in checkpoints)
assert all(point['truth_xy_error_m']<=.20 and point['truth_yaw_error_rad']<=.10 for point in checkpoints)
assert turn>=.35 and lateral>=.5 and state['occupied']>0 and active
assert abs(o.twist.twist.linear.x)<=.05 and abs(o.twist.twist.angular.z)<=.10
assert rtf>=.8
PY
cat "$logs/metrics.json"
printf 'FAST-LIO house Nav2 acceptance smoke passed.\n'
