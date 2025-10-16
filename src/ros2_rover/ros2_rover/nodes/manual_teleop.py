# manual_teleop.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys, select, termios, tty

class ManualTeleop(Node):
    """
    Simple keyboard teleop to switch mission_mode back to manual (emits to mission_manager).
    Press 'm' to set manual, 's' for science, 'd' for delivery, 'e' equipment, 'a' autonav.
    """
    def __init__(self):
        super().__init__('manual_teleop')
        self.pub = self.create_publisher(String, '/mission/mode', 10)
        self.get_logger().info("Manual teleop started: keys m,s,d,e,a to switch modes")
        self.timer = self.create_timer(0.2, self.poll_keyboard)

    def poll_keyboard(self):
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            c = sys.stdin.read(1)
            mapping = {'m':'manual','s':'science','d':'delivery','e':'equipment','a':'autonav'}
            if c in mapping:
                msg = String()
                msg.data = mapping[c]
                self.pub.publish(msg)
                self.get_logger().info(f"Published mission mode: {msg.data}")

def main(args=None):
    # set terminal to raw
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    rclpy.init()
    node = ManualTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    node.destroy_node()
    rclpy.shutdown()
