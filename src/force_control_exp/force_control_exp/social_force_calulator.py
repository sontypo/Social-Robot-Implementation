import rclpy
import rclpy.executors
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseArray, Twist
import rclpy.qos
from std_msgs.msg import Int16MultiArray, Float32MultiArray 
from visualization_msgs.msg import Marker, MarkerArray
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
import threading
import math
import numpy as np
from tf_transformations import euler_from_quaternion

class SocialForceCalculator(Node):
    def __init__(self):
        super().__init__('social_force_calulator') # type: ignore
        self.robot_pose = Point()
        self.robot_velocity = Twist()
        self.object_linear_vel = Twist().linear
        
        self.declare_parameter('tau_d', 2.953125)
        self.tau_d = self.get_parameter('tau_d').get_parameter_value().double_value    # original: 0.2953125 - it was Ali's params
        
        self.declare_parameter('sigma', 38.8623)
        self.sigma = self.get_parameter('sigma').get_parameter_value().double_value    # for computing new v_des
        
        self.declare_parameter('Lambda', 0.04375)
        self.Lambda = self.get_parameter('Lambda').get_parameter_value().double_value
        
        # Force manitude
        self.declare_parameter('alpha', 83.518066)
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value
        
        # Force range
        self.declare_parameter('beta', 1.806738)
        self.beta = self.get_parameter('beta').get_parameter_value().double_value
        
        # designated velocity 
        self.F_soc = np.array([0.0, 0.0])
        self.theta = 0.0

        # Subscriber for the robot's position 
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Subscriber for the object's position (topic /person in this case)
        # self.create_subscription(MarkerArray, '/person', self.object_position_callback, 10)
        self.object_positions = {}  # Store the previous positions and timestamps of object
        
        # self.create_subscription(Float32MultiArray, '/person_array', self.object_position_callback, 10)
        self.create_subscription(PoseArray, '/person_pose_array', self.human_pose_callback, 10)
        
        # Subscriber for the object's position (an object in gazebo, "actor" in this case)
        # self.create_subscription(ModelStates, '/gazebo/model_states', self.actor_pose_callback, 10)
        # self.actor_name = 'actor_1'

        # Publisher for the applied force
        self.social_force_publisher = self.create_publisher(Float32MultiArray, '/social_force', 10)
                
        # define the previous robot position
        self.robot_prev_pose = None
        
        # Initialize variables to store previous position and time
        self.previous_human_positions = []
        self.prev_obj_position = None
        self.prev_time = self.get_clock().now().seconds_nanoseconds()[0]
        
    
    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose.position
        self.robot_velocity = msg.twist.twist
        orientation_q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        
    
    # The object's anisotropic behavior function
    def gamma_function(self, Lambda, cos_phi_ij):
        return Lambda + 0.5 * (1 - Lambda) * (1 + cos_phi_ij)
        
        
    def match_poses_by_proximity(self, current_pose):
        """Match current human poses to previous poses based on proximity."""
        matched_poses = []
        closest_prev_pose = None
        min_distance = float('inf')

        # Find the closest previous pose
        for prev_pose in self.previous_human_positions:
            distance = self.calculate_distance(current_pose.position, prev_pose.position)
            if distance < min_distance:
                min_distance = distance
                closest_prev_pose = prev_pose

        if closest_prev_pose is not None:
            matched_poses.append((closest_prev_pose, current_pose))

        return matched_poses

    def calculate_distance(self, pos1, pos2):
        """Calculate the Euclidean distance between two 3D points."""
        return math.sqrt(
            (pos2.x - pos1.x) ** 2 +
            (pos2.y - pos1.y) ** 2 +
            (pos2.z - pos1.z) ** 2
        )
        
    def human_pose_callback(self, human_pose_msg: PoseArray):
        # F_soc Calculation    
        if len(human_pose_msg.poses) != 0:
            self.get_logger().info('Human detected !!!')
            current_time = self.get_clock().now().seconds_nanoseconds()[0]

            if len(self.previous_human_positions) == 0:
                self.previous_human_positions = human_pose_msg.poses
            
            delta_time = current_time - self.prev_time
            F_soc = np.array([0.0, 0.0])
            self.F_soc = np.array([0.0, 0.0])
            for i, human_pose in enumerate(human_pose_msg.poses):
                # Match current poses to previous poses based on proximity
                matched_poses = self.match_poses_by_proximity(human_pose)
                # Calculate velocities for matched pairs
                for prev_pose, curr_pose in matched_poses:
                    # Calculate velocity (change in position over time)                    
                    delta_x = curr_pose.position.x - prev_pose.position.x
                    delta_y = curr_pose.position.y - prev_pose.position.y
                    delta_t = (current_time - self.prev_time)

                    if delta_t <= 0:
                        # self.get_logger().warn('Time interval must be positive')
                        return

                    # Calculate velocity components
                    human_vel_x = delta_x / delta_t
                    human_vel_y = delta_y / delta_t
                    
                    # Compute the linear velocity of the robot relative to the object
                    # TODO: This should be in robot frame, so there is no linear.y velocity
                    vec_relative_vel = np.array([self.robot_velocity.linear.x - human_vel_x, 0.0 - human_vel_y]) # TODO: Checked
                    
                    # Compute the vectorized distance between the robot and the object
                    # TODO: This should probably only be computed by the human position (the relative position to the robot), no need to refer to the origin
                    vec_dis = np.array([-curr_pose.position.x, -curr_pose.position.y]) # TODO: Checked
                    
                    # Encounter angle's cosine calculation
                    cos_phi_ij = (vec_relative_vel / np.linalg.norm(vec_relative_vel)) * (-vec_dis / np.linalg.norm(vec_dis))
                    if np.isnan(cos_phi_ij[0]):
                        cos_phi_ij = np.array([0.0, 0.0])
                    
                    # Compute the object's anisotropic behavior
                    Gamma = self.gamma_function(self.Lambda, cos_phi_ij)
                    
                    # Compute the distance between the robot and the object
                    # TODO: This should probably only be computed by the human position (the relative position to the robot), no need to refer to the origin
                    distance = math.sqrt((curr_pose.position.x)**2 + (curr_pose.position.y)**2) # TODO: Checked
                                                    
                    # Compute the social force applied to the mobile robot
                    F_soc = Gamma * self.alpha * math.exp(-distance / self.beta) * (vec_dis / distance)

                    if math.isnan(F_soc[0]):
                        F_soc = np.array([0.0, 0.0])
                        
                    print("- F_soc: ", F_soc)
                    
                    # Log the velocity for this person
                    # self.get_logger().info(f'Velocity of person: vx={human_vel_x:.2f}, vy={human_vel_y:.2f}')

                self.F_soc += F_soc
            # Update the previous time and positions
            self.prev_time = current_time
            self.previous_human_positions = human_pose_msg.poses
        else: 
            self.get_logger().info('No human detected !!!')
            self.F_soc = np.array([0.0, 0.0])
            
        # Create and publish the force message
        f_soc_msg = Float32MultiArray()
        f_soc_msg.data.append(self.F_soc[0])
        f_soc_msg.data.append(self.F_soc[1])
        self.social_force_publisher.publish(f_soc_msg)
        
        
def main():
    rclpy.init()
    socialForce = SocialForceCalculator()

    # Create an instance of MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    # Add the node to the executor
    executor.add_node(socialForce)

    try:
        rclpy.spin(socialForce)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Disabling F_soc...")
    finally:
        # Clean up and shutdown
        executor.shutdown()
        socialForce.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()