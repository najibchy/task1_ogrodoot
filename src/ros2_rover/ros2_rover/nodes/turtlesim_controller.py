# turtlesim_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from turtlesim.msg import Pose as TurtlePose
from math import sqrt, atan2
from rclpy.duration import Duration

class TurtlesimController(Node):
    """
    Subscribes to /turtle/waypoint_cmd (PoseStamped) and moves turtle
    via publishing to /turtle1/cmd_vel. Also subscribers turtlesim pose
    to compute arrival.
    """
    def __init__(self):
        super().__init__('turtlesim_controller')
        self.waypoint_sub = self.create_subscription(PoseStamped, '/turtle/waypoint_cmd', self.on_waypoint, 10)
        self.pose_sub = self.create_subscription(TurtlePose, '/turtle1/pose', self.on_pose, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.current_pose = None
        self.target = None
        self.declare_parameter('waypoint_tolerance', 0.25)

        # timer to drive motion
        self.timer = self.create_timer(0.1, self.control_loop)

    def on_waypoint(self, msg):
        self.target = msg.pose
        self.get_logger().info(f"Received waypoint: {self.target.position.x}, {self.target.position.y}")

    def on_pose(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if self.target is None or self.current_pose is None:
            return
        tx = self.target.position.x
        ty = self.target.position.y
        cx = self.current_pose.x
        cy = self.current_pose.y
        dx = tx - cx
        dy = ty - cy
        dist = sqrt(dx*dx + dy*dy)
        tol = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        twist = Twist()
        if dist > tol:
            # simple proportional controller
            angle_to_target = atan2(dy, dx)
            # map to turtle's orientation: turtlesim uses theta
            # rotate and move by combining linear.x and angular.z
            twist.linear.x = min(2.0 * dist, 2.0)
            twist.angular.z = 4.0 * (angle_to_target - self.current_pose.theta)
            self.cmd_pub.publish(twist)
        else:
            # stop
            self.cmd_pub.publish(Twist())
            # clear target so we stop commanding
            self.target = None
            self.get_logger().info("Reached waypoint")

def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
