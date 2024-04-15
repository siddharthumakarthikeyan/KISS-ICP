import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import time

class OdometryLogger(Node):
    def __init__(self):
        super().__init__('odometry_logger')
        self.odometry_sub = self.create_subscription(
            Odometry,
            '/kiss/odometry',
            self.odometry_callback,
            10)
        self.target_sub = self.create_subscription(
            Int32,
            '/target_reached',
            self.target_callback,
            10)
        self.ready_to_log = False

    def target_callback(self, msg):
        if msg.data == 1:
            self.ready_to_log = True
            self.get_logger().info("Received signal. Ready to log next odometry data.")

    def odometry_callback(self, msg):
        if self.ready_to_log:
            with open('odometry_once.log', 'a') as log_file:
                translation = msg.pose.pose.position
                orientation = msg.pose.pose.orientation
                log_file.write(f"Translation: [x: {translation.x}, y: {translation.y}, z: {translation.z}]\n")
                log_file.write(f"Orientation: [x: {orientation.x}, y: {orientation.y}, z: {orientation.z}, w: {orientation.w}]\n")
                log_file.write("\n")
            self.get_logger().info("Odometry data logged once. Shutting down...")
            self.ready_to_log = False
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    odometry_logger = OdometryLogger()
    try:
        rclpy.spin(odometry_logger)
    except KeyboardInterrupt:
        pass
    finally:
        odometry_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

