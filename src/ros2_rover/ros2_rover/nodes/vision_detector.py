# vision_detector.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Point

class VisionDetector(Node):
    """
    Subscribes to /camera/image_raw and performs simple color-based detection.
    Publishes detection messages on /vision/detections as simple string messages
    and offers a service-like topic via is_visible (we'll just publish detection messages).
    """
    def __init__(self):
        super().__init__('vision_detector')
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.pub = self.create_publisher(String, '/vision/detections', 10)
        self.bridge = CvBridge()
        # color thresholds for 'red_circle' and 'green_box' targets
        self.targets = {
            'red_circle': {'hsv_lower': (0, 120, 70), 'hsv_upper': (10, 255, 255)},
            'green_box': {'hsv_lower': (40, 40, 40), 'hsv_upper': (80, 255, 255)}
        }

    def on_image(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        detections = []
        for label, params in self.targets.items():
            lower = np.array(params['hsv_lower'])
            upper = np.array(params['hsv_upper'])
            mask = cv2.inRange(hsv, lower, upper)
            # morphological ops
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                # take largest contour
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > 200:  # threshold
                    M = cv2.moments(c)
                    if M['m00'] != 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                    else:
                        cx, cy = 0, 0
                    detections.append((label, cx, cy, area))
        # publish detections
        for det in detections:
            s = String()
            s.data = f"{det[0]}:{det[1]},{det[2]},{det[3]}"
            self.pub.publish(s)

def main(args=None):
    rclpy.init(args=args)
    node = VisionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
