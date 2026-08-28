"""Display the robot's current high-level state in RViz/Foxglove."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


STATE_COLORS = {
    'IDLE': (0.25, 0.65, 1.0),
    'VALIDATING': (0.55, 0.45, 1.0),
    'NAVIGATING': (0.05, 0.85, 0.95),
    'WAITING_FOR_BASE_STATIONARY': (1.0, 0.72, 0.10),
    'PERCEIVING': (0.75, 0.35, 1.0),
    'PLANNING_GRASP': (1.0, 0.45, 0.08),
    'EXECUTING_GRASP': (1.0, 0.18, 0.08),
    'VERIFYING_GRASP': (0.95, 0.80, 0.10),
    'HOLDING_OBJECT': (0.10, 0.90, 0.30),
    'SUCCEEDED': (0.10, 0.90, 0.30),
    'FAILED': (1.0, 0.05, 0.05),
    'CANCELLED': (0.55, 0.55, 0.55),
    'STOPPING': (1.0, 0.08, 0.08),
}


class StateVisualizer(Node):
    def __init__(self):
        super().__init__('state_visualizer')
        self.state = 'IDLE'
        self.publisher = self.create_publisher(MarkerArray, 'development/robot_state_markers', 10)
        self.create_subscription(String, 'robot_state', self.on_state, 10)
        self.create_timer(0.25, self.publish_markers)

    def on_state(self, message):
        self.state = message.data.strip().upper() or 'IDLE'
        self.get_logger().info(f'visual state: {self.state}')

    def publish_markers(self):
        red, green, blue = STATE_COLORS.get(self.state, (1.0, 1.0, 1.0))
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        text = Marker()
        text.header.frame_id = 'base_link'
        text.header.stamp = now
        text.ns = 'robot_state'
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = -0.02
        text.pose.position.z = 0.62
        text.pose.orientation.w = 1.0
        text.scale.z = 0.11
        text.color.r, text.color.g, text.color.b, text.color.a = red, green, blue, 1.0
        text.text = f'STATE: {self.state}'
        markers.markers.append(text)

        halo = Marker()
        halo.header.frame_id = 'base_link'
        halo.header.stamp = now
        halo.ns = 'robot_state'
        halo.id = 1
        halo.type = Marker.CYLINDER
        halo.action = Marker.ADD
        halo.pose.position.z = 0.012
        halo.pose.orientation.w = 1.0
        halo.scale.x = halo.scale.y = 0.43
        halo.scale.z = 0.012
        halo.color.r, halo.color.g, halo.color.b, halo.color.a = red, green, blue, 0.28
        markers.markers.append(halo)

        self.publisher.publish(markers)


def main():
    rclpy.init()
    node = StateVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
