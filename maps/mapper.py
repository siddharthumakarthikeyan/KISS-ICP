import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32
import numpy as np
import open3d as o3d
import time

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.local_map_sub = self.create_subscription(
            PointCloud2,
            '/kiss/local_map',
            self.map_callback,
            10)
        self.target_reached_sub = self.create_subscription(
            Int32,
            '/target_reached',
            self.target_callback,
            10)
        self.map_count = 0
        self.ready_to_save = False
        self.start_time = time.time()

    def target_callback(self, msg):
        if msg.data == 1:
            self.get_logger().info("Target reached, ready to save map.")
            self.ready_to_save = True

    def map_callback(self, msg):
        if self.ready_to_save:
            points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
            xyz = points[:, :3]
            colors = np.zeros_like(xyz)
            colors[:, 0] = (xyz[:, 0] - np.min(xyz[:, 0])) / (np.max(xyz[:, 0]) - np.min(xyz[:, 0]))
            colors[:, 1] = (xyz[:, 1] - np.min(xyz[:, 1])) / (np.max(xyz[:, 1]) - np.min(xyz[:, 1]))
            colors[:, 2] = (xyz[:, 2] - np.min(xyz[:, 2])) / (np.max(xyz[:, 2]) - np.min(xyz[:, 2]))

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz)
            pcd.colors = o3d.utility.Vector3dVector(colors)

            file_name = f"local_map_{self.map_count}.pcd"
            o3d.io.write_point_cloud(file_name, pcd)
            self.map_count += 1
            self.get_logger().info(f"Saved map to {file_name}")

            self.get_logger().info("Map saved, shutting down...")
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    map_saver = MapSaver()
    rclpy.spin(map_saver)
    map_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

