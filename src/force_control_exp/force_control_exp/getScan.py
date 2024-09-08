import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np

class SideDistanceComputer(Node):
    def __init__(self):
        super().__init__('side_distance_computer')
        self.distances_publisher = self.create_publisher(Float32MultiArray, 'scan_distances', 10)
        
        self.create_subscription(
            LaserScan,
            '/scan',  # Update with the actual LIDAR topic name
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        left_indices = range(480, 600)  # Indices corresponding to angles around +π/2 (left side)
        right_indices = range(120, 240)  # Indices corresponding to angles around -π/2 (right side)
        
        # Extract the minimum distance for each side
        left_distance = min(msg.ranges[i] for i in left_indices if msg.ranges[i] > msg.range_min)
        right_distance = min(msg.ranges[i] for i in right_indices if msg.ranges[i] > msg.range_min)
        
        distances = np.array([left_distance, right_distance])
        
        # Publish the distances
        self.distances_publisher.publish(Float32MultiArray(data=distances))
        
        self.get_logger().info(f'Left distance: {left_distance}, Right distance: {right_distance}')

def main(args=None):
    rclpy.init(args=args)
    side_distance_computer = SideDistanceComputer()
    rclpy.spin(side_distance_computer)
    side_distance_computer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
