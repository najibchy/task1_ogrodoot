# autonav_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from ros2_rover.ros2_rover import config  # not used, but config read path below
import yaml
import os
import time

class AutonavNode(Node):
    """
    Autonomous navigator: visits a sequence of waypoints (gnss and vision types).
    Prints LED statuses to the terminal as logs.
    """
    def __init__(self):
        super().__init__('autonav_node')
        self.mode_sub = self.create_subscription(String, '/mission/mode', self.on_mode, 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/turtle/waypoint_cmd', 10)
        self.vision_sub = self.create_subscription(String, '/vision/detections', self.on_vision, 10)
        self.pending_vision = set()
        self.declare_parameter('waypoints_file', os.path.join(os.getcwd(), 'ros2_rover', 'config', 'waypoints.yaml'))

    def on_vision(self, msg):
        # record that vision target seen
        # format label:cx,cy,area
        parts = msg.data.split(':')
        if len(parts) >= 2:
            label = parts[0]
            self.pending_vision.add(label)

    def on_mode(self, msg):
        if msg.data == 'autonav':
            self.get_logger().info('LED: RED (Autonomous)')
            self.execute_autonav()

    def execute_autonav(self):
        path = self.get_parameter('waypoints_file').get_parameter_value().string_value
        with open(path, 'r') as f:
            cfg = yaml.safe_load(f)
        seq = cfg.get('autonav_sequence', [])
        for i, item in enumerate(seq):
            pose = PoseStamped()
            pose.pose.position.x = float(item['x'])
            pose.pose.position.y = float(item['y'])
            self.waypoint_pub.publish(pose)
            self.get_logger().info(f"Sent autonav waypoint {i+1}: {pose.pose.position.x},{pose.pose.position.y}")
            # wait heuristics to reach
            time.sleep(4.0)
            if item.get('type') == 'vision':
                label = item.get('label')
                # wait up to 6 seconds for vision detection
                wait_until = time.time() + 6.0
                while time.time() < wait_until:
                    if label in self.pending_vision:
                        self.get_logger().info(f"Vision target {label} reached")
                        self.pending_vision.discard(label)
                        break
                    time.sleep(0.5)
                else:
                    self.get_logger().warn(f"Vision target {label} not seen (timeout)")
            self.get_logger().info('LED: GREEN (Target Reached)')
            time.sleep(0.6)
        self.get_logger().info('Autonav finished. LED: BLUE (Manual)')
        self.get_logger().info('LED: BLUE (Manual)')

def main(args=None):
    rclpy.init(args=args)
    node = AutonavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
