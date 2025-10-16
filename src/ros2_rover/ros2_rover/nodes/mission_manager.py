# mission_manager.py
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
import json
import os

class MissionManager(Node):
    """
    Controls the active mission mode. Exposes a dynamic parameter 'mission_mode'
    and publishes the current mode on /mission/mode. Modes: manual, science, delivery,
    equipment, autonav
    """
    def __init__(self):
        super().__init__('mission_manager')
        self.declare_parameter('mission_mode', 'manual')
        self.mode_pub = self.create_publisher(String, '/mission/mode', 10)

        # listen for parameter changes
        self.add_on_set_parameters_callback(self.param_change_callback)

        # publish initial
        self.publish_mode()

    def param_change_callback(self, params):
        for p in params:
            if p.name == 'mission_mode' and p.type_ == Parameter.Type.STRING:
                self.get_logger().info(f"Mission mode changing to: {p.value}")
                self.publish_mode(p.value)
        return rclpy.parameter.ParameterEventDescriptor()  # allow change

    def publish_mode(self, mode=None):
        if mode is None:
            mode = self.get_parameter('mission_mode').get_parameter_value().string_value
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f"Published mission mode: {mode}")

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
