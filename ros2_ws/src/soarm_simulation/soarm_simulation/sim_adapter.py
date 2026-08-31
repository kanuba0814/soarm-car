"""ROS compatibility seam for the chassis-only Kuruma Fortress model."""
import math
import struct

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TransformStamped
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

from soarm_interfaces.action import GoToPlaceAndGrasp


class SimAdapter(Node):
    def __init__(self):
        super().__init__('kuruma_sim_adapter')
        self.declare_parameter('publish_odometry', True)
        self.declare_parameter('arm_enabled', False)
        self.publish_odometry = self.get_parameter('publish_odometry').value
        self.arm_enabled = self.get_parameter('arm_enabled').value
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20) if self.publish_odometry else None
        self.power_pub = self.create_publisher(Float32, '/PowerVoltage', 5)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 50)
        self.points_pub = self.create_publisher(PointCloud2, '/mid360/points', 10)
        self.livox_pub = self.create_publisher(CustomMsg, '/livox/lidar', 10)
        self.livox_imu_pub = self.create_publisher(Imu, '/livox/imu', 50)
        self.tf = TransformBroadcaster(self) if self.publish_odometry else None
        if self.publish_odometry:
            self.create_subscription(Odometry, '/gazebo/odom', self.on_gazebo_odom, 20)
        self.create_subscription(Imu, '/gazebo/imu', self.on_imu, 50)
        self.create_subscription(Imu, '/gazebo/livox_imu', self.on_livox_imu, 50)
        self.create_subscription(PointCloud2, '/mid360/scan/points/points', self.on_points, 10)
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

    def on_gazebo_odom(self, source):
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
