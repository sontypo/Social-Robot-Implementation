import math
import rclpy
import rclpy.executors
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import rclpy.qos
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np
import math
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray

class SideDistanceComputer(Node):
    def __init__(self):
        super().__init__('side_distance_computer')
        self.distances_publisher = self.create_publisher(Float32MultiArray, 'scan_distances', 10)
        
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,\
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,\
                                          depth=1)
        
        self.create_subscription(LaserScan, '/scan', self.get_scan, qos_profile=qos_policy)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publishers for left and right side markers
        self.left_marker_publisher = self.create_publisher(MarkerArray, 'left_markers', 10)
        self.right_marker_publisher = self.create_publisher(MarkerArray, 'right_markers', 10)
        self.theta = 0.0
        
        
    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose.position
        self.robot_velocity = msg.twist.twist
        orientation_q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        

    def get_scan(self, scan: LaserScan):
        deg_theta = math.floor((self.theta * 180) / math.pi)
        calib_samples = deg_theta * 2
        
        left_indices = range(480 - calib_samples, 600 - calib_samples)  # Indices corresponding to angles around +π/2 (left side)
        right_indices = range(120 - calib_samples, 240 - calib_samples)  # Indices corresponding to angles around -π/2 (right side)
        
        scan_range = []

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(5.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])
        
        self.left_distance = min(scan_range[i] for i in left_indices if scan_range[i] > scan.range_min)
        self.right_distance = min(scan_range[i] for i in right_indices if scan_range[i] > scan.range_min)
        
        # Extract points for both sides and create markers
        left_markers = self.create_markers(scan, left_indices, 'left_marker_ns', 0)
        right_markers = self.create_markers(scan, right_indices, 'right_marker_ns', 1000)
        
        # Publish the markers
        self.left_marker_publisher.publish(left_markers)
        self.right_marker_publisher.publish(right_markers)
        
    def create_markers(self, scan, indices, ns, marker_id_offset):
        """Create markers for the LIDAR scan for the given indices."""
        marker_array = MarkerArray()
        for idx, i in enumerate(indices):
            if scan.ranges[i] >= scan.range_min and scan.ranges[i] <= scan.range_max:
                # Calculate the angle of the point
                angle = scan.angle_min + i * scan.angle_increment
                # Calculate the Cartesian coordinates of the point
                x = scan.ranges[i] * np.cos(angle)
                y = scan.ranges[i] * np.sin(angle)

                # Create a marker for this point
                marker = Marker()
                marker.header.frame_id = 'base_link'  # Update as needed (typically your robot's frame)
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = ns
                marker.id = marker_id_offset + idx
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.0  # 2D LIDAR, so z is 0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.05  # Diameter of the sphere (adjust as needed)
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.a = 1.0  # Opacity of the marker
                marker.color.r = 0.0  # Color (adjust as needed)
                marker.color.g = 1.0
                marker.color.b = 1.0
                
                marker_array.markers.append(marker) # type: ignore

        return marker_array

def main(args=None):
    rclpy.init(args=args)
    side_distance_computer = SideDistanceComputer()
    # Create an instance of MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    # Add the node to the executor
    executor.add_node(side_distance_computer)

    try:
        rclpy.spin(side_distance_computer)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Stopping Dynamic Updater...")
    finally:
        # Clean up and shutdown
        executor.shutdown()
        side_distance_computer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
