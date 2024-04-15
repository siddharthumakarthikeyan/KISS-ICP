import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class PointCloudRemapper(Node):
    def __init__(self):
        super().__init__('point_cloud_remapper')
        self.subscription = self.create_subscription(
            PointCloud2,
            'points_raw',
            self.point_cloud_callback,
            10)
        self.publisher = self.create_publisher(
            PointCloud2,
            'velodyne_points',
            10)

    def point_cloud_callback(self, msg):
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    point_cloud_remapper = PointCloudRemapper()
    rclpy.spin(point_cloud_remapper)
    point_cloud_remapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

