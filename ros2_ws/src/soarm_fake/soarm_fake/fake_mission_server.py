"""Visualization-only request preview; it never commands or simulates hardware."""

import time

import rclpy
import yaml
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from soarm_interfaces.action import GoToPlaceAndGrasp


class VisualizationPreview(Node):
    def __init__(self):
        super().__init__('visualization_preview')
        self.declare_parameter('policy_file', '')
        with open(self.get_parameter('policy_file').value, encoding='utf-8') as stream:
            policy = yaml.safe_load(stream)

        self.places = set(policy['input']['allowed_places'])
        self.objects = set(policy['input']['allowed_objects'])
        self.selected_place = ''
        self.selected_object = ''
        self.markers = self.create_publisher(MarkerArray, 'development/scene', 10)
        self.create_timer(0.5, self.publish_scene)
        self.server = ActionServer(
            self,
            GoToPlaceAndGrasp,
            'go_to_place_and_grasp',
            self.execute,
            goal_callback=self.validate_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.get_logger().info(
            'Visualization preview ready; requests are displayed but navigation and grasp are not simulated.'
        )

    def validate_goal(self, request):
        valid = (
            bool(request.request_id)
            and request.place_id in self.places
            and request.object_id in self.objects
        )
        return GoalResponse.ACCEPT if valid else GoalResponse.REJECT

    def execute(self, handle):
        self.selected_place = handle.request.place_id
        self.selected_object = handle.request.object_id
        self.publish_scene()

        feedback = GoToPlaceAndGrasp.Feedback()
        feedback.stage = 'visualization_preview'
        feedback.detail = 'request displayed only; no navigation or grasp executed'
        feedback.progress_percent = 100
        handle.publish_feedback(feedback)
        time.sleep(0.2)

        result = GoToPlaceAndGrasp.Result()
        result.request_id = handle.request.request_id
        result.final_stage = 'visualization_preview'
        result.success = False
        result.reason = 'preview accepted; hardware execution is intentionally unavailable'
        handle.succeed()
        return result

    def publish_scene(self):
        markers = MarkerArray()
        scene = [
            (0, 'robot', 0.0, 0.0, 0.2, 0.7, 1.0, 0.60, 0.40, 0.20),
            (1, 'demo_table', 1.5, 0.0, 0.2, 1.0, 0.2, 0.70, 0.70, 0.12),
            (2, 'demo_cube', 1.5, 0.0, 1.0, 0.4, 0.1, 0.12, 0.12, 0.12),
        ]
        now = self.get_clock().now().to_msg()
        for marker_id, label, x, y, red, green, blue, sx, sy, sz in scene:
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = now
            marker.ns = 'development_scene'
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = sz / 2.0
            marker.pose.orientation.w = 1.0
            marker.scale.x, marker.scale.y, marker.scale.z = sx, sy, sz
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = red, green, blue, 0.9
            markers.markers.append(marker)

            text = Marker()
            text.header.frame_id = 'base_link'
            text.header.stamp = now
            text.ns = 'development_labels'
            text.id = marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = sz + 0.15
            text.pose.orientation.w = 1.0
            text.scale.z = 0.12
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            selected = label in (self.selected_place, self.selected_object)
            text.text = f'{label}{" [selected]" if selected else ""}'
            markers.markers.append(text)
        self.markers.publish(markers)


def main():
    rclpy.init()
    node = VisualizationPreview()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
