"""ROS action/task seam that commands and reads the native Fortress arm joints."""
import math
import threading
import time

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from std_msgs.msg import Float32, Float64
from tf2_ros import TransformBroadcaster

from soarm_interfaces.action import GoToPlaceAndGrasp

ARM_JOINTS = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex', 'arm_wrist_flex', 'arm_wrist_roll']
GRIPPER_JOINTS = ['arm_gripper_opening']
ALL_ARM_JOINTS = ARM_JOINTS + GRIPPER_JOINTS


class SimAdapter(Node):
    def __init__(self):
        super().__init__('senior_akm_sim_adapter')
        self.declare_parameter('policy_file', '')
        self.declare_parameter('publish_odometry', True)
        self.policy = self._load_policy(self.get_parameter('policy_file').value)
        self.gazebo_joints = {}
        self.resource_lock = threading.Lock()
        self.base_busy = False
        self.arm_busy = False
        self.hold_targets = {name: 0.0 for name in ALL_ARM_JOINTS}
        self.publish_odometry = self.get_parameter('publish_odometry').value
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20) if self.publish_odometry else None
        self.command_pub = self.create_publisher(Twist, '/cmd_vel', 20)
        self.power_pub = self.create_publisher(Float32, '/PowerVoltage', 5)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 50)
        self.points_pub = self.create_publisher(PointCloud2, '/mid360/points', 10)
        self.livox_pub = self.create_publisher(CustomMsg, '/livox/lidar', 10)
        self.livox_imu_pub = self.create_publisher(Imu, '/livox/imu', 50)
        self.joint_commands = {
            name: self.create_publisher(Float64, f'/gazebo/{name}/cmd_pos', 10)
            for name in ALL_ARM_JOINTS
        }
        self.tf = TransformBroadcaster(self) if self.publish_odometry else None
        if self.publish_odometry:
            self.create_subscription(Odometry, '/gazebo/odom', self.on_gazebo_odom, 20)
        self.create_subscription(Imu, '/gazebo/imu', self.on_imu, 50)
        self.create_subscription(Imu, '/gazebo/livox_imu', self.on_livox_imu, 50)
        self.create_subscription(PointCloud2, '/mid360/scan/points', self.on_points, 10)
        # /joint_states is bridged directly from Fortress JointStatePublisher. This callback is
        # deliberately only a cache for Action completion, never a synthetic state publisher.
        self.create_subscription(JointState, '/joint_states', self.on_gazebo_joint_state, 20)
        self.create_timer(1.0, lambda: self.power_pub.publish(Float32(data=24.0)))
        # Real servos hold their commanded zero pose after power-on. Reproduce that boundary
        # instead of letting the native joints fall under gravity before the first Action goal.
        self.create_timer(0.1, self.publish_hold_targets)
        action_callbacks = ReentrantCallbackGroup()
        self.arm_server = ActionServer(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory',
            self.execute_trajectory, goal_callback=lambda request: self.trajectory_goal(request, ARM_JOINTS),
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=action_callbacks)
        self.gripper_server = ActionServer(
            self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory',
            self.execute_trajectory, goal_callback=lambda request: self.trajectory_goal(request, GRIPPER_JOINTS),
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=action_callbacks)
        self.mission_server = ActionServer(
            self, GoToPlaceAndGrasp, '/go_to_place_and_grasp', self.execute_mission,
            goal_callback=self.mission_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=action_callbacks)

    @staticmethod
    def _load_policy(path):
        if not path:
            return {'input': {'allowed_places': ['demo_table'], 'allowed_objects': ['demo_cube']}, 'places': {'demo_table': [1.2, 0.0]}}
        with open(path, encoding='utf-8') as source:
            return yaml.safe_load(source)

    def on_gazebo_odom(self, source):
        if not self.publish_odometry:
            return
        odom = Odometry()
        odom.header.stamp = source.header.stamp
        odom.header.frame_id = 'odom_combined'
        odom.child_frame_id = 'base_footprint'
        odom.pose = source.pose
        odom.twist = source.twist
        self.odom_pub.publish(odom)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)

    def on_imu(self, source):
        source.header.frame_id = 'gyro_link'
        self.imu_pub.publish(source)

    def on_livox_imu(self, source):
        # This is a separate IMU collocated with the simulated LiDAR, not gyro_link.
        source.header.frame_id = 'livox_frame'
        self.livox_imu_pub.publish(source)

    def on_points(self, source):
        """Preserve Fortress GPU LiDAR geometry and adapt it to FAST-LIO's CustomMsg ABI.

        Fortress PointCloudPacked has no per-return time.  Offset times are therefore a
        deterministic linear 100 ms scan approximation, rather than MID-360 firing times.
        """
        fields = {field.name: field for field in source.fields}
        required = {'x', 'y', 'z'}
        xyz_fields = [fields[name] for name in required if name in fields]
        valid_xyz = (len(xyz_fields) == 3 and source.point_step > 0 and
                     all(field.datatype == PointField.FLOAT32 and
                         field.offset + 4 <= source.point_step for field in xyz_fields))
        if source.is_bigendian or not valid_xyz:
            self.get_logger().warning(
                'MID-360 PointCloud2 requires little-endian float32 x/y/z fields; dropping scan')
            return
        import struct
        points = []
        intensity_field = fields.get('intensity')
        if intensity_field is not None and not (
                intensity_field.datatype == PointField.FLOAT32 and
                intensity_field.offset + 4 <= source.point_step):
            intensity_field = None
        for row in range(source.height):
            for column in range(source.width):
                base = row * source.row_step + column * source.point_step
                if base + source.point_step > len(source.data):
                    break
                x, y, z = (struct.unpack_from('<f', source.data, base + fields[name].offset)[0]
                           for name in ('x', 'y', 'z'))
                if not all(math.isfinite(value) for value in (x, y, z)):
                    continue
                reflectivity = 128
                if intensity_field is not None:
                    intensity = struct.unpack_from('<f', source.data, base + intensity_field.offset)[0]
                    if math.isfinite(intensity):
                        reflectivity = max(0, min(255, round(intensity)))
                points.append((x, y, z, reflectivity))

        # Keep the original native point cloud available for inspection/debugging.
        source.header.frame_id = 'livox_frame'
        self.points_pub.publish(source)
        message = CustomMsg()
        message.header = source.header
        message.timebase = source.header.stamp.sec * 1_000_000_000 + source.header.stamp.nanosec
        message.point_num = len(points)
        message.lidar_id = 0
        message.points = [CustomPoint(
            offset_time=0 if len(points) < 2 else index * 100_000_000 // (len(points) - 1),
            x=x, y=y, z=z, reflectivity=reflectivity, tag=0x10, line=index % 4)
            for index, (x, y, z, reflectivity) in enumerate(points)]
        self.livox_pub.publish(message)

    def on_gazebo_joint_state(self, source):
        self.gazebo_joints.update(zip(source.name, source.position))

    def publish_hold_targets(self):
        for name, position in self.hold_targets.items():
            self.joint_commands[name].publish(Float64(data=position))

    def trajectory_goal(self, request, expected_joints):
        names = request.trajectory.joint_names
        valid = bool(names) and set(names) == set(expected_joints) and bool(request.trajectory.points)
        valid = valid and all(len(point.positions) == len(names) for point in request.trajectory.points)
        if not valid:
            return GoalResponse.REJECT
        with self.resource_lock:
            if self.arm_busy:
                return GoalResponse.REJECT
            self.arm_busy = True
        return GoalResponse.ACCEPT

    def release_arm(self):
        with self.resource_lock:
            self.arm_busy = False

    def hold_arm(self):
        names = [name for name in ALL_ARM_JOINTS if name in self.gazebo_joints]
        self.command_positions(names, [self.gazebo_joints[name] for name in names])

    def stop_base(self):
        for _ in range(5):
            self.command_pub.publish(Twist())
            time.sleep(0.02)

    def command_positions(self, names, positions):
        self.hold_targets.update(zip(names, positions))
        # Re-publish briefly so the ROS->Gazebo bridge and native controller have time to
        # discover this one-shot Action command; no state is synthesized here.
        messages = [Float64(data=position) for position in positions]
        for _ in range(5):
            for name, message in zip(names, messages):
                self.joint_commands[name].publish(message)
            time.sleep(0.02)

    def wait_for_positions(self, names, positions, timeout, cancelled=None):
        deadline = time.monotonic() + timeout
        messages = [Float64(data=position) for position in positions]
        while time.monotonic() < deadline:
            if cancelled and cancelled():
                return False
            if all(name in self.gazebo_joints and
                   abs(self.gazebo_joints[name] - position) < (0.02 if name in GRIPPER_JOINTS else 0.04)
                   for name, position in zip(names, positions)):
                return True
            # Position commands are stateful in Fortress. Re-publishing also covers the
            # initial ROS-to-Gazebo discovery window without implementing a controller.
            for name, message in zip(names, messages):
                self.joint_commands[name].publish(message)
            time.sleep(0.05)
        return False

    def execute_trajectory(self, handle):
        try:
            trajectory = handle.request.trajectory
            started = time.monotonic()
            for point in trajectory.points:
                due = started + point.time_from_start.sec + point.time_from_start.nanosec / 1e9
                while time.monotonic() < due:
                    if handle.is_cancel_requested:
                        self.hold_arm()
                        handle.canceled()
                        return FollowJointTrajectory.Result()
                    time.sleep(0.02)
                self.command_positions(trajectory.joint_names, point.positions)
                # Fortress / DART does not provide a stable contact-capable simulation for this
                # coarse single-servo gripper. Preserve its standard Action seam, but only claim
                # physical position convergence for the five arm joints.
                if set(trajectory.joint_names) == set(GRIPPER_JOINTS):
                    continue
                if not self.wait_for_positions(trajectory.joint_names, point.positions, 20.0,
                                               lambda: handle.is_cancel_requested):
                    if handle.is_cancel_requested:
                        self.hold_arm()
                        handle.canceled()
                    else:
                        handle.abort()
                    return FollowJointTrajectory.Result()
            handle.succeed()
            return FollowJointTrajectory.Result()
        finally:
            self.release_arm()

    def mission_goal(self, request):
        valid = bool(request.request_id) and request.place_id in self.policy['input']['allowed_places'] and request.object_id in self.policy['input']['allowed_objects']
        if not valid:
            return GoalResponse.REJECT
        with self.resource_lock:
            if self.base_busy or self.arm_busy:
                return GoalResponse.REJECT
            self.base_busy = True
            self.arm_busy = True
        return GoalResponse.ACCEPT

    def execute_mission(self, handle):
        try:
            feedback = GoToPlaceAndGrasp.Feedback()
            feedback.stage = 'moving'
            feedback.detail = 'Fortress Ackermann drive; deterministic demo motion, not navigation'
            feedback.progress_percent = 20
            handle.publish_feedback(feedback)
            command = Twist()
            command.linear.x = 0.15
            end = time.monotonic() + 7.7
            while time.monotonic() < end:
                if handle.is_cancel_requested:
                    self.stop_base()
                    self.hold_arm()
                    handle.canceled()
                    return self._mission_result(handle, False, 'cancelled; base stopped and arm held')
                self.command_pub.publish(command)
                time.sleep(0.05)
            self.stop_base()
            for stage, percent in [('grasping_contact_approximation', 75), ('lifting', 90)]:
                feedback.stage = stage
                feedback.detail = 'native Fortress arm joints; no MoveIt and no object attach plugin'
                feedback.progress_percent = percent
                handle.publish_feedback(feedback)
                target = [0.0] if stage.startswith('grasping') else [-0.45]
                names = GRIPPER_JOINTS if stage.startswith('grasping') else ['arm_shoulder_lift']
                self.command_positions(names, target)
                if names == GRIPPER_JOINTS:
                    continue
                if not self.wait_for_positions(names, target, 20.0, lambda: handle.is_cancel_requested):
                    if handle.is_cancel_requested:
                        self.stop_base()
                        self.hold_arm()
                        handle.canceled()
                        return self._mission_result(handle, False, 'cancelled; base stopped and arm held')
                    handle.abort()
                    return self._mission_result(handle, False, 'Gazebo arm joint did not reach target')
            handle.succeed()
            return self._mission_result(handle, True, 'simulated motion completed; object grasp not verified')
        finally:
            self.stop_base()
            with self.resource_lock:
                self.base_busy = False
                self.arm_busy = False

    @staticmethod
    def _mission_result(handle, success, reason):
        result = GoToPlaceAndGrasp.Result()
        result.success = success
        result.request_id = handle.request.request_id
        result.final_stage = 'complete' if success else 'cancelled'
        result.reason = reason
        return result


def main():
    rclpy.init()
    node = SimAdapter()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
