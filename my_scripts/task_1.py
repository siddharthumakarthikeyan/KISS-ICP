import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import os
import pykitti 

class KITTILidarStreamer(Node):
    def __init__(self):
        super().__init__('kitti_lidar_streamer')
        self.publisher_ = self.create_publisher(PointCloud2, 'raw_points', 10)
        self.timer_period = 0.1 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        

        base_dir = ""
        date = ""
        drive = ""
        self.data = pykitti.raw(base_dir, date, drive)

        self.velodyne_generator = self.data.velo
        
    def timer_callback(self):
        try:
            scan = next(self.velodyne_generator)
            scan = np.array(scan, dtype=np.float32)

            msg = PointCloud2()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "velodyne"
            msg.height = 1
            msg.width = len(scan)

            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
            ]

            msg.is_bigendian = False
            msg.point_step = 16  # float32 (4 bytes) * 4
            msg.row_step = msg.point_step * msg.width
            msg.data = scan.tobytes()

            self.publisher_.publish(msg)
        except StopIteration:
            self.get_logger().info("Reached the end of the velodyne data.")
            self.destroy_timer(self.timer)  # Optionally stop the timer

def main(args=None):
    rclpy.init(args=args)
    kitti_lidar_streamer = KITTILidarStreamer()
    rclpy.spin(kitti_lidar_streamer)
    kitti_lidar_streamer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

