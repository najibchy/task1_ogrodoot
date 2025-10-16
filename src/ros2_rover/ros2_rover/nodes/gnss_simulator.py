# gnss_simulator.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from turtlesim.msg import Pose as TurtlePose
from ros2_rover_interfaces.srv import CaptureImage  # for type usage? not needed here
from std_srvs.srv import Trigger
import time

class GnssSimulator(Node):
    """
    Publishes a simple NavSatFix computed by mapping turtlesim x,y to lat/lon.
    Also offers service /gnss/get_fix which returns last fix in a small custom format
    (we'll reuse NavSatFix in response by publishing to topic only).
    """
    def __init__(self):
        super().__init__('gnss_simulator')
        self.pub = self.create_publisher(NavSatFix, '/gnss/fix', 10)
        self.pose_sub = self.create_subscription(TurtlePose, '/turtle1/pose', self.on_pose, 10)
        self.last_fix = None
        self.declare_parameter('gnss_scale', 0.0001)  # scale from turtlesim coords
        self.timer = self.create_timer(0.5, self.publish_fix)

    def on_pose(self, msg):
        # store for mapping
        self.current_pose = msg

    def publish_fix(self):
        if not hasattr(self, 'current_pose'):
            return
        scale = self.get_parameter('gnss_scale').get_parameter_value().double_value
        # simple affine transform: lat = base + y*scale, lon = base + x*scale
        base_lat = 28.0
        base_lon = 89.0
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.latitude = base_lat + (self.current_pose.y * scale)
        fix.longitude = base_lon + (self.current_pose.x * scale)
        fix.altitude = 0.0
        self.pub.publish(fix)
        self.last_fix = fix

def main(args=None):
    rclpy.init(args=args)
    node = GnssSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
