# equipment_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from std_msgs.msg import String
import time

class EquipmentNode(Node):
    """
    Simulates servicing equipment. Watches /mission/mode and runs a service-like sequence.
    """
    def __init__(self):
        super().__init__('equipment_node')
        self.mode_sub = self.create_subscription(String, '/mission/mode', self.on_mode, 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/turtle/waypoint_cmd', 10)

    def on_mode(self, msg):
        if msg.data == 'equipment':
            self.get_logger().info("Starting equipment service mission")
            # read a sample location (hard-coded for simplicity)
            pose = PoseStamped()
            pose.pose.position.x = 5.0
            pose.pose.position.y = 5.0
            self.waypoint_pub.publish(pose)
            self.get_logger().info("Moving to equipment site")
            # wait
            time.sleep(4.0)
            # simulate checks
            ok = self.perform_checks()
            if ok:
                self.get_logger().info("Equipment serviced OK")
            else:
                self.get_logger().warn("Equipment service failed")

    def perform_checks(self):
        # simulate a diagnostic with 95% success
        import random
        return random.random() > 0.05

def main(args=None):
    rclpy.init(args=args)
    node = EquipmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
