# science_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ros2_rover_interfaces.srv import CaptureImage
from sensor_msgs.msg import NavSatFix
import csv
import os
import yaml
from std_msgs.msg import String

class ScienceNode(Node):
    """
    Accepts start commands by watching /mission/mode. When mission_mode == 'science'
    it loads waypoints from config and visits each waypoint by publishing to /turtle/waypoint_cmd.
    On arrival it calls the camera capture service and logs GNSS + image path in CSV.
    """
    def __init__(self):
        super().__init__('science_node')
        self.mode_sub = self.create_subscription(String, '/mission/mode', self.on_mode, 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/turtle/waypoint_cmd', 10)
        self.camera_cli = self.create_client(CaptureImage, '/camera/capture_image')
        self.gnss_sub = self.create_subscription(NavSatFix, '/gnss/fix', self.on_gnss, 10)
        self.current_fix = None
        self.declare_parameter('waypoints_file', os.path.join(os.getcwd(), 'ros2_rover', 'config', 'waypoints.yaml'))
        self.running = False

    def on_gnss(self, msg):
        self.current_fix = msg

    def on_mode(self, msg):
        if msg.data == 'science' and not self.running:
            self.get_logger().info("Starting science mission")
            self.running = True
            self.execute_mission()

    def execute_mission(self):
        # load waypoints from yaml
        path = self.get_parameter('waypoints_file').get_parameter_value().string_value
        with open(path, 'r') as f:
            cfg = yaml.safe_load(f)
        wps = cfg.get('science_waypoints', [])
        logfolder = os.path.join(os.getcwd(), 'logs', 'science')
        os.makedirs(logfolder, exist_ok=True)
        csvpath = os.path.join(logfolder, 'manifest.csv')
        with open(csvpath, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['image_path','timestamp','gnss_lat','gnss_lon','turtle_x','turtle_y'])
            for i, wp in enumerate(wps):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = float(wp['x'])
                pose.pose.position.y = float(wp['y'])
                self.waypoint_pub.publish(pose)
                self.get_logger().info(f"Sent waypoint {i+1}: {wp['x']},{wp['y']}")
                # wait until turtlesim_controller announces "Reached waypoint" via log; simple wait loop
                # We'll wait a fixed time (tunable) then capture image
                self.get_logger().info("Waiting 4s for arrival (simple heuristic)")
                self._spin_wait(4.0)
                # call camera capture
                if not self.camera_cli.wait_for_service(timeout_sec=2.0):
                    self.get_logger().warn("Camera service not available, skipping image capture")
                    continue
                req = CaptureImage.Request()
                req.filename_prefix = f"science_wp{i+1}"
                req.use_webcam = False
                fut = self.camera_cli.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
                if fut.result() is not None and fut.result().success:
                    res = fut.result()
                    lat = self.current_fix.latitude if self.current_fix else 0.0
                    lon = self.current_fix.longitude if self.current_fix else 0.0
                    # turtlesim x,y are approximate, include them as GNSS mapping inverse if needed
                    writer.writerow([res.filepath, res.timestamp, lat, lon, wp['x'], wp['y']])
                    self.get_logger().info(f"Logged {res.filepath} with GNSS {lat},{lon}")
                else:
                    self.get_logger().warn("Image capture failed or timed out")
        self.get_logger().info(f"Science mission finished, manifest at {csvpath}")
        self.running = False

    def _spin_wait(self, duration):
        # spin the node for duration seconds to allow callbacks
        end = self.get_clock().now().seconds_nanoseconds()[0] + duration
        while self.get_clock().now().seconds_nanoseconds()[0] < end:
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = ScienceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
