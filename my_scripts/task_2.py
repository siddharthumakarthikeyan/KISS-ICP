import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32
import csv

class ContinuousVelodyneDataChecker(Node):
    def __init__(self, csv_file_path):
        super().__init__('continuous_velodyne_data_checker')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            Int32,
            '/target_reached',
            10)  
        self.csv_data = self.read_csv_data(csv_file_path)

    def read_csv_data(self, csv_file_path):
        with open(csv_file_path, 'r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                return [int(i) for i in row]
        return []

    def listener_callback(self, msg):
        incoming_data_list = list(msg.data)
        if incoming_data_list == self.csv_data:
            self.get_logger().info('Target reached')
            self.publisher.publish(Int32(data=1))  # Publish '1' when target is reached
        else:
            self.get_logger().info('Processing')

def main(args=None):
    rclpy.init(args=args)
    csv_file_path = '/home/siddharth/ati_motors/my_scripts/velodyne_data.csv'
    continuous_checker = ContinuousVelodyneDataChecker(csv_file_path)
    rclpy.spin(continuous_checker)
    continuous_checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

