import math
import struct
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from .core import TemporalMap

class UafTmNode(Node):
    def __init__(self):
        super().__init__('uaf_tm_node')
        self.declare_parameter('depth_topic', '/camera/depth_image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('odom_topic', '/wheel/odom_raw')
        self.declare_parameter('map_resolution', 0.10)
        self.declare_parameter('max_depth_m', 8.0)
        self.info = None; self.pose = (0., 0., 0.)
        self.map = TemporalMap(self.get_parameter('map_resolution').value)
        self.points_pub = self.create_publisher(PointCloud2, '/uaf_tm/points', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/uaf_tm/map', 1)
        self.odom_pub = self.create_publisher(Odometry, '/uaf_tm/odom', 10)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value, self.info_cb, 10)
        self.create_subscription(Image, self.get_parameter('depth_topic').value, self.depth_cb, 5)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_cb, 10)

    def info_cb(self, msg): self.info = msg
    def odom_cb(self, msg):
        p = msg.pose.pose.position; q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        self.pose = (p.x, p.y, yaw); self.odom_pub.publish(msg)

    def depth_cb(self, msg):
        if self.info is None or msg.width == 0: return
        fx, fy, cx, cy = self.info.k[0], self.info.k[4], self.info.k[2], self.info.k[5]
        if fx <= 0.0 or fy <= 0.0: return
        step = max(1, int(max(msg.width, msg.height) / 80)); points = []
        unit = 0.001 if msg.encoding in ('16UC1', 'mono16') else 1.0
        for v in range(0, msg.height, step):
            for u in range(0, msg.width, step):
                offset = v * msg.step + u * (2 if unit < 1 else 4)
                try:
                    depth = struct.unpack_from('<H' if unit < 1 else '<f', msg.data, offset)[0] * unit
                except (struct.error, IndexError): continue
                if not math.isfinite(depth) or depth <= 0.1 or depth > self.get_parameter('max_depth_m').value: continue
                points.append((depth, -(u-cx) * depth / fx))
        self.map.update(points, self.pose, 1.0 if len(points) > 20 else 0.4)
        self.publish_points(msg.header, points)
        self.publish_map(msg.header)

    def publish_points(self, header, points):
        fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1) for i,n in enumerate(('x','y','z'))]
        data = b''.join(struct.pack('<fff', x, y, 0.) for x,y in points)
        out = PointCloud2(header=header, height=1, width=len(points), fields=fields, is_bigendian=False, point_step=12, row_step=12*len(points), data=data, is_dense=False)
        self.points_pub.publish(out)

    def publish_map(self, header):
        occupied = self.map.occupied(); msg = OccupancyGrid(); msg.header = header; msg.header.frame_id = 'odom'
        msg.info.resolution = self.map.resolution; msg.info.width = 160; msg.info.height = 160
        msg.info.origin.position.x = -8.0; msg.info.origin.position.y = -8.0
        msg.data = [100 if (column - 80, row - 80) in occupied else 0
                    for row in range(160) for column in range(160)]
        self.map_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args); node = UafTmNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
