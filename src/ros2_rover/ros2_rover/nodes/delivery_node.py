# delivery_node.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from ros2_rover_interfaces.action import PickDeliver
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
from ros2_rover_interfaces.srv import CaptureImage

class DeliveryActionServer(Node):
    """
    Implements an action server /delivery/pick_and_deliver that receives pickup/dropoff poses.
    """
    def __init__(self):
        super().__init__('delivery_node')
        self._action_server = ActionServer(self, PickDeliver, 'delivery/pick_and_deliver', self.execute_callback, goal_callback=self.goal_callback, cancel_callback=self.cancel_callback)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/turtle/waypoint_cmd', 10)
        self.payload_pub = self.create_publisher(String, '/payload/status', 10)

    def goal_callback(self, goal_request):
        self.get_logger().info('Received delivery goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        feedback_msg = PickDeliver.Feedback()
        # move to pickup
        feedback_msg.stage = 'moving_to_pickup'
        feedback_msg.percent = 0.0
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info('Moving to pickup')
        self._goto_pose(goal.pickup)
        feedback_msg.percent = 40.0
        goal_handle.publish_feedback(feedback_msg)
        # simulate pickup
        self.payload_pub.publish(String(data='attached'))
        self.get_logger().info('Picked up payload')
        feedback_msg.stage = 'picked'
        feedback_msg.percent = 60.0
        goal_handle.publish_feedback(feedback_msg)
        # move to dropoff
        feedback_msg.stage = 'moving_to_dropoff'
        goal_handle.publish_feedback(feedback_msg)
        self._goto_pose(goal.dropoff)
        feedback_msg.percent = 90.0
        goal_handle.publish_feedback(feedback_msg)
        # deliver
        self.payload_pub.publish(String(data='delivered'))
        self.get_logger().info('Delivered payload')
        result = PickDeliver.Result()
        result.success = True
        result.message = 'Delivered OK'
        goal_handle.succeed()
        return result

    def _goto_pose(self, pose_stamped):
        # publish waypoint and wait a small amount (in real world you'd check pose)
        self.waypoint_pub.publish(pose_stamped)
        # wait heuristic
        time.sleep(4.0)

def main(args=None):
    rclpy.init(args=args)
    node = DeliveryActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
