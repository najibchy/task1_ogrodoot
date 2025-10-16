# camera_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ros2_rover_interfaces.srv import CaptureImage
import cv2
from cv_bridge import CvBridge
import os
from datetime import datetime
import numpy as np

class CameraNode(Node):
    """
    Publishes camera frames (dummy or webcam) and provides a service /camera/capture_image
    that saves a frame to disk and returns the filepath.
    """
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('use_webcam', False)
        self.declare_parameter('dummy_image_dir', '')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = None
        self.frame = None
        self.timer = self.create_timer(0.1, self.publish_frame)
        self.srv = self.create_service(CaptureImage, '/camera/capture_image', self.capture_callback)

        self.use_webcam = self.get_parameter('use_webcam').get_parameter_value().bool_value
        self.dummy_dir = self.get_parameter('dummy_image_dir').get_parameter_value().string_value

        if self.use_webcam:
            self.cap = cv2.VideoCapture(0)

    def publish_frame(self):
        if self.use_webcam and self.cap is not None:
            ret, frame = self.cap.read()
            if not ret:
                return
        else:
            # generate a simple dummy image (colored gradient)
            h, w = 480, 640
            frame = np.zeros((h, w, 3), dtype=np.uint8)
            cv2.putText(frame, "DUMMY CAMERA", (50,240), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 3)
        self.frame = frame
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(msg)

    def capture_callback(self, request, response):
        # save current frame
        prefix = request.filename_prefix if request.filename_prefix else 'capture'
        ts = datetime.utcnow().strftime('%Y%m%d_%H%M%S')
        folder = os.path.join(os.getcwd(), 'logs', 'images')
        os.makedirs(folder, exist_ok=True)
        filename = f"{prefix}_{ts}.jpg"
        path = os.path.join(folder, filename)
        if self.frame is None:
            response.success = False
            response.filepath = ''
            response.timestamp = ''
            return response
        cv2.imwrite(path, self.frame)
        response.success = True
        response.filepath = path
        response.timestamp = ts
        self.get_logger().info(f"Saved image to {path}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node.cap is not None:
        node.cap.release()
    node.destroy_node()
    rclpy.shutdown()
