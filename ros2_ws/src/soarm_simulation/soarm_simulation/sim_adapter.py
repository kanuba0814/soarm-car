"""ROS compatibility seam for the chassis-only Kuruma Fortress model."""
import math
import struct
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TransformStamped, Twist
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, PointCloud2
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

from soarm_interfaces.action import GoToPlaceAndGrasp


class SimAdapter(Node):
    def __init__(self):
        super().__init__('kuruma_sim_adapter')
        self.declare_parameter('publish_odometry', True)
        self.declare_parameter('arm_enabled', False)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('points_input_topic', '/mid360/scan/points/points')
        self.publish_odometry = self.get_parameter('publish_odometry').value
        self.arm_enabled = self.get_parameter('arm_enabled').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.wheel_radius = 0.0763
        self.kinematic_radius = 0.5 * (0.3472 + 0.4456)
        self.wheel_names = (
            'front_left_wheel_joint', 'front_right_wheel_joint',
            'rear_left_wheel_joint', 'rear_right_wheel_joint',
        )
        self.wheel_positions = None
        self.wheel_stamp = None
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20) if self.publish_odometry else None
        self.ground_truth_pub = self.create_publisher(Odometry, '/ground_truth/odom', 20) if self.publish_odometry else None
        self.safe_cmd_pub = self.create_publisher(Twist, '/safe_cmd_vel', 10)
        self.last_command_time = None
        self.command_active = False
        self.power_pub = self.create_publisher(Float32, '/PowerVoltage', 5)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 50)
        self.points_pub = self.create_publisher(PointCloud2, '/mid360/points', 10)
        self.livox_pub = self.create_publisher(CustomMsg, '/livox/lidar', 10)
        self.livox_imu_pub = self.create_publisher(Imu, '/livox/imu', 50)
        self.tf = TransformBroadcaster(self) if self.publish_odometry else None
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(0.05, self.stop_stale_command)
        if self.publish_odometry:
            self.create_subscription(Odometry, '/gazebo/odom', self.on_gazebo_odom, 20)
            self.create_subscription(JointState, '/joint_states', self.on_joint_states, 50)
        self.create_subscription(Imu, '/gazebo/imu', self.on_imu, 50)
        self.create_subscription(Imu, '/gazebo/livox_imu', self.on_livox_imu, 50)
        self.create_subscription(
            PointCloud2, self.get_parameter('points_input_topic').value,
            self.on_points, 10)
        self.create_timer(1.0, lambda: self.power_pub.publish(Float32(data=24.0)))
        # Interfaces remain discoverable, but chassis-first simulation never fabricates arm success.
        self.arm_server = ActionServer(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory', self.unavailable_trajectory, goal_callback=self.arm_goal, cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.gripper_server = ActionServer(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory', self.unavailable_trajectory, goal_callback=self.arm_goal, cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.mission_server = ActionServer(self, GoToPlaceAndGrasp, '/go_to_place_and_grasp', self.unavailable_mission, goal_callback=self.mission_goal, cancel_callback=lambda _: CancelResponse.ACCEPT)

    def arm_goal(self, _request):
        self.get_logger().warning('rejecting arm trajectory: arm_enabled=false; no arm is installed')
        return GoalResponse.REJECT

    def mission_goal(self, _request):
        self.get_logger().warning('rejecting grasp mission: arm_enabled=false; no arm is installed')
        return GoalResponse.REJECT

    def unavailable_trajectory(self, handle):
        handle.abort()
        return FollowJointTrajectory.Result()

    def unavailable_mission(self, handle):
        result = GoToPlaceAndGrasp.Result()
        result.success = False
        result.request_id = handle.request.request_id
        result.final_stage = 'arm_not_installed'
        result.reason = 'arm_enabled=false; chassis-first simulation cannot execute grasp missions'
        handle.abort()
        return result

    def on_cmd_vel(self, command):
        self.safe_cmd_pub.publish(command)
        self.last_command_time = time.monotonic()
        self.command_active = any(abs(value) > 1e-9 for value in (
            command.linear.x, command.linear.y, command.angular.z,
        ))

    def stop_stale_command(self):
        if not self.command_active or self.last_command_time is None:
            return
        if time.monotonic() - self.last_command_time <= self.command_timeout:
            return
        self.safe_cmd_pub.publish(Twist())
        self.command_active = False
        self.get_logger().warning('cmd_vel timed out; commanded all wheels to stop')

    def on_gazebo_odom(self, source):
        ground_truth = Odometry()
        ground_truth.header.stamp = source.header.stamp
        ground_truth.header.frame_id = 'world'
        ground_truth.child_frame_id = 'base_footprint'
        ground_truth.pose = source.pose
        ground_truth.twist = source.twist
        self.ground_truth_pub.publish(ground_truth)

    def on_joint_states(self, source):
        indices = {name: index for index, name in enumerate(source.name)}
        if not all(name in indices for name in self.wheel_names):
            return
        wheel_indices = tuple(indices[name] for name in self.wheel_names)
        if not source.position or max(wheel_indices) >= len(source.position):
            return
        positions = tuple(source.position[index] for index in wheel_indices)
        stamp = source.header.stamp.sec + source.header.stamp.nanosec * 1e-9
        if not all(math.isfinite(value) for value in positions) or not math.isfinite(stamp):
            return
        if self.wheel_positions is None or self.wheel_stamp is None:
            self.wheel_positions = positions
            self.wheel_stamp = stamp
            return
        dt = stamp - self.wheel_stamp
        previous_positions = self.wheel_positions
        self.wheel_positions = positions
        self.wheel_stamp = stamp
        if dt <= 0.0 or dt > 0.5:
            return

        fl, fr, rl, rr = (
            positions[index] - previous_positions[index] for index in range(4)
        )
        body_dx = self.wheel_radius * (fl + fr + rl + rr) * 0.25
        body_dy = self.wheel_radius * (-fl + fr + rl - rr) * 0.25
        body_dyaw = self.wheel_radius * (-fl + fr - rl + rr) / (4.0 * self.kinematic_radius)
        heading = self.odom_yaw + 0.5 * body_dyaw
        self.odom_x += math.cos(heading) * body_dx - math.sin(heading) * body_dy
        self.odom_y += math.sin(heading) * body_dx + math.cos(heading) * body_dy
        self.odom_yaw += body_dyaw

        odom = Odometry()
        odom.header = source.header
        odom.header.frame_id = 'odom_combined'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation.z = math.sin(0.5 * self.odom_yaw)
        odom.pose.pose.orientation.w = math.cos(0.5 * self.odom_yaw)
        odom.twist.twist.linear.x = body_dx / dt
        odom.twist.twist.linear.y = body_dy / dt
        odom.twist.twist.angular.z = body_dyaw / dt
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[7] = 0.01
        odom.twist.covariance[35] = 0.02
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = self.odom_x
        transform.transform.translation.y = self.odom_y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)

    def on_imu(self, source):
        source.header.frame_id = 'gyro_link'
        self.imu_pub.publish(source)

    def on_livox_imu(self, source):
        source.header.frame_id = 'livox_frame'
        self.livox_imu_pub.publish(source)

    def on_points(self, source):
        fields = {field.name: field for field in source.fields}
        if source.is_bigendian or not {'x', 'y', 'z'} <= fields.keys() or source.point_step <= 0:
            return
        points = []
        for index in range(source.width * source.height):
            offset = index * source.point_step
            if offset + source.point_step > len(source.data):
                break
            xyz = [struct.unpack_from('<f', source.data, offset + fields[name].offset)[0] for name in ('x', 'y', 'z')]
            if all(math.isfinite(value) for value in xyz):
                points.append(xyz)
        source.header.frame_id = 'livox_frame'
        self.points_pub.publish(source)
        message = CustomMsg()
        message.header = source.header
        message.timebase = source.header.stamp.sec * 1_000_000_000 + source.header.stamp.nanosec
        message.point_num = len(points)
        message.lidar_id = 0
        message.points = [CustomPoint(offset_time=0 if len(points) < 2 else index * 100_000_000 // (len(points) - 1), x=x, y=y, z=z, reflectivity=128, tag=0x10, line=index % 4) for index, (x, y, z) in enumerate(points)]
        self.livox_pub.publish(message)


def main():
    rclpy.init()
    node = SimAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
