import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import csv

class VelodyneSubscriber(Node):
    def __init__(self):
        super().__init__('velodyne_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.listener_callback,
            10)
        self.file = open("velodyne_data.csv", "w", newline='')  # Open a CSV file to write the data
        self.csv_writer = csv.writer(self.file)  # Create a CSV writer object
        self.received = False  # Flag to ensure only one message is processed

    def listener_callback(self, msg):
        if not self.received:  # Check if data has not been captured yet
            data_list = list(msg.data)
            self.csv_writer.writerow(data_list)  # Write data as a row in the CSV file
            self.file.close()  # Close the file after writing
            self.received = True  # Set the flag to True after capturing data
            self.get_logger().info("Data captured and written to CSV file, shutting down...")
            rclpy.shutdown()  # Shutdown after capturing a single scan

def main(args=None):
    rclpy.init(args=args)
    velodyne_subscriber = VelodyneSubscriber()
    rclpy.spin(velodyne_subscriber)
    velodyne_subscriber.destroy_node()

if __name__ == '__main__':
    main()

