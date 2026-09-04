"""ROS adapter: navigation only uses Nav2 NavigateToPose; it never publishes Twist."""
import math, time
from .core import State
try:
 import rclpy
 from rclpy.action import ActionClient
 from rclpy.node import Node
 from nav2_msgs.action import NavigateToPose
 from geometry_msgs.msg import PoseStamped
except ImportError: rclpy=None
class RoomSearchNode(Node if rclpy else object):
 def __init__(self):
  super().__init__('room_search')
  self.declare_parameter('map_id','');self.declare_parameter('localization_healthy',False);self.declare_parameter('sensor_timeout',1.0)
  self.client=ActionClient(self,NavigateToPose,'navigate_to_pose');self.state=State.EMPTY;self.goal=None;self.last_sensor=0.0
  self.create_timer(.25,self._safety_tick)
 def navigate(self,x,y,yaw):
  """Send pose goals only after caller's GUI/core intent confirmation."""
  if not self.get_parameter('localization_healthy').value or not self.get_parameter('map_id').value:
   self.block('localization map_id or health not ready');return
  goal=NavigateToPose.Goal();goal.pose=PoseStamped();goal.pose.header.frame_id='map';goal.pose.header.stamp=self.get_clock().now().to_msg();goal.pose.pose.position.x=x;goal.pose.pose.position.y=y;goal.pose.pose.orientation.z=math.sin(yaw/2);goal.pose.pose.orientation.w=math.cos(yaw/2)
  self.goal=self.client.send_goal_async(goal);self.state=State.NAVIGATING_TO_SCAN
 def scan_headings(self,x,y,starting_yaw):
  """Fixed XY heading goals, no segment exceeds pi/2."""
  return [(x,y,starting_yaw+i*math.pi/2) for i in range(1,5)]
 def sensor_received(self): self.last_sensor=time.monotonic()
 def block(self,reason):
  if self.goal: self.goal.cancel()
  self.state=State.BLOCKED;self.get_logger().error('BLOCKED: '+reason)
 def _safety_tick(self):
  if self.state in (State.NAVIGATING_TO_SCAN,State.NAVIGATING_TO_DOCK,State.SCANNING):
   if not self.get_parameter('localization_healthy').value or not self.get_parameter('map_id').value:self.block('localization unhealthy or map_id absent')
   elif self.last_sensor and time.monotonic()-self.last_sensor>float(self.get_parameter('sensor_timeout').value):self.block('sensor stale')
def main():
 if rclpy is None: raise RuntimeError('ROS 2 Humble/rclpy is unavailable; use room_search_gui -- simulation')
 rclpy.init();node=RoomSearchNode()
 try:rclpy.spin(node)
 finally:node.destroy_node();rclpy.shutdown()
