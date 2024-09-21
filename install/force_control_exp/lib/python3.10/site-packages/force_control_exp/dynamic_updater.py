import rclpy
import rclpy.executors
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist
import rclpy.qos
from std_msgs.msg import Int16MultiArray, Float32MultiArray 
from visualization_msgs.msg import Marker, MarkerArray
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from tf_transformations import euler_from_quaternion

class DynamicUpdater(Node):
    def __init__(self):
        super().__init__('dynamic_updater') # type: ignore
        self.robot_pose = Point()
        self.robot_velocity = Twist()
        self.object_linear_vel = Twist().linear
        
        self.declare_parameter('using_lidar', True)
        self.using_lidar = self.get_parameter('using_lidar').get_parameter_value().bool_value
        
        self.declare_parameter('total_mass', 25.0)
        self.total_mass = self.get_parameter('total_mass').get_parameter_value().double_value
        
        self.declare_parameter('m_wheel', 2.5)
        m_wheel = self.get_parameter('m_wheel').get_parameter_value().double_value
        
        self.declare_parameter('R_wheel', 0.08)
        R_wheel = self.get_parameter('R_wheel').get_parameter_value().double_value
        
        self.declare_parameter('cam2com', 0.2)
        self.cam2com = self.get_parameter('cam2com').get_parameter_value().double_value #m
        
        self.declare_parameter('v_max_robot', 2.5)
        self.v_max_robot = self.get_parameter('v_max_robot').get_parameter_value().double_value
        
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
        
        self.declare_parameter('alpha_bound', 100.0)
        self.alpha_bound = self.get_parameter('alpha_bound').get_parameter_value().double_value
        
        self.declare_parameter('beta_bound', 3.5)
        self.beta_bound = self.get_parameter('beta_bound').get_parameter_value().double_value
        
        self.declare_parameter('boundary_dis', 3.15)
        self.boundary_dis = self.get_parameter('boundary_dis').get_parameter_value().double_value
        
        m_base = 20.0
        self.I_w = 1/2 * m_wheel * R_wheel*R_wheel
        # self.I_b = 1/12 * m_base * (0.26*0.26 + 0.13*0.13 + 0.605*0.605)  # 0.416 + 0.8
        self.I_b = 1.216
        
        # designated velocity 
        # self.v_des = np.array([self.v_max_robot, 0])      # this one is for the previous version
        self.v_des = np.array([0.0, 0.0])   # new version of v_des for computing F_des
        self.F_des = np.array([0.0, 0.0])
        self.F_soc = np.array([0.0, 0.0])
        self.F_total = np.array([0.0, 0.0])
        self.theta = 0.0
        # Added boundary force's terms
        self.F_bound = np.array([0.0, 0.0])
        
        # Define conversion matrix
        # for designated force
        self.des_conversion = np.array([ [math.cos(self.theta), 0], [0, -math.sin(self.theta)] ])
        # for boundary force
        self.bound_conversion = np.array([ [0, math.sin(self.theta)], [0, math.cos(self.theta)] ])

        # Subscriber for the robot's position 
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Subscriber for the object's position (topic /person in this case)
        # self.create_subscription(MarkerArray, '/person', self.object_position_callback, 10)
        self.object_positions = {}  # Store the previous positions and timestamps of object
        
        self.create_subscription(Float32MultiArray, '/person_array', self.object_position_callback, 10)

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,\
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,\
                                          depth=1)
        
        self.create_subscription(LaserScan, '/scan', self.get_scan, qos_profile=qos_policy)
        
        # Publishers for left and right side markers
        self.left_marker_publisher = self.create_publisher(MarkerArray, 'left_markers', 10)
        self.right_marker_publisher = self.create_publisher(MarkerArray, 'right_markers', 10)
        
        # Subscriber for the object's position (an object in gazebo, "actor" in this case)
        # self.create_subscription(ModelStates, '/gazebo/model_states', self.actor_pose_callback, 10)
        # self.actor_name = 'actor_1'

        # Publisher for the applied force
        self.force_publisher = self.create_publisher(Float32MultiArray, '/applied_force', 10)
        
        # Publisher for the velocity commands
        self.velocity_command = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.time = self.get_clock().now()
        self.pre_time = self.get_clock().now()
        
        # define the previous robot position
        self.robot_prev_pose = None
        # define the distance threshold for bounding force update
        self.diff_thres = 0.25 #m
        self.left_distance = self.boundary_dis / 2
        self.right_distance = self.boundary_dis / 2
        
        # Initialize variables to store previous position and time
        self.prev_obj_position = None
        self.prev_time = None
        
    
    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.pre_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        self.pre_time = current_time
        Fm = self.F_total[0]
        Fn = self.F_total[1]
        v_x = self.robot_velocity.linear.x
        omega_z = self.robot_velocity.angular.z
        dynamic_velocity = self.update_dynamic(Fm, Fn, v_x, omega_z, self.theta, dt)
                
        # Create the public velocity msg
        vel_msg = Twist()
        vel_msg.linear.x = dynamic_velocity[0]
        vel_msg.angular.z = dynamic_velocity[1]
        self.velocity_command.publish(vel_msg)
        
        
    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose.position
        self.robot_velocity = msg.twist.twist
        orientation_q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        if self.robot_prev_pose is None:
            self.robot_prev_pose = self.robot_pose
            return
        
        
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
    
    
    # The object's anisotropic behavior function
    def gamma_function(self, Lambda, cos_phi_ij):
        return Lambda + 0.5 * (1 - Lambda) * (1 + cos_phi_ij)
    

    def object_position_callback(self, markers_msg):
        current_time = self.get_clock().now()
        if self.robot_pose is None:
            self.get_logger().warn('Waiting for odometry callback...')
            return
        
        if self.robot_prev_pose is None:
            self.robot_prev_pose = self.robot_pose
            return
        
        # Define conversion matrix
        # for designated force
        self.des_conversion = np.array([ [math.cos(self.theta), 0], [-math.sin(self.theta), 0] ])
        # for boundary force
        self.bound_conversion = np.array([ [0, math.sin(self.theta)], [0, math.cos(self.theta)] ])
        # for conversion F_total from global to robot frame
        self.total_conversion = np.array([ [math.cos(self.theta), math.sin(self.theta)], [-math.sin(self.theta), math.cos(self.theta)] ])
        
        pose_diff = math.sqrt((self.robot_prev_pose.x - self.robot_pose.x)**2 + (self.robot_prev_pose.y - self.robot_pose.y)**2)
        # Compute the boundary force applied to the mobile robot
        if self.using_lidar:
            self.get_logger().info('Ultilizing LIDAR sensor')
            self.get_logger().debug(self.cam2com)
            # Compute the boundary forces from both sides
            # This is Global frame
            self.F_bound_L = np.array( [0, self.alpha_bound * math.exp(-self.left_distance / self.beta_bound)] )
            
            self.F_bound_R = np.array( [0, self.alpha_bound * math.exp(-self.right_distance / self.beta_bound)] )
            
            # Total boundary force
            self.F_bound = self.F_bound_R - self.F_bound_L
            
            # We have to convert the boundary force to the robot frame
            self.F_bound = np.dot( self.bound_conversion, (self.F_bound_R - self.F_bound_L) )
            
        else:
            self.get_logger().info('Using default boundary distance')
            # if pose_diff >= self.diff_thres:
            ''' Assume that the mobile robot is always tends to be positioned at the center of the hallway, which is the intial position of it.
            The boundary forces applied to the robot are from both sides, the distance between it and the wall is determined as: d2bound_L & d2bound_R '''
            d2bound_L = (self.boundary_dis / 2) - self.robot_pose.y
            d2bound_R = (self.boundary_dis / 2) - (-self.robot_pose.y)
            # Compute the boundary forces from both sides
            # This is Global frame
            self.F_bound_L = np.array( [0, self.alpha_bound * math.exp(-d2bound_L / self.beta_bound)] )
            
            self.F_bound_R = np.array( [0, self.alpha_bound * math.exp(-d2bound_R / self.beta_bound)] )
            
            # Total boundary force
            self.F_bound = self.F_bound_R - self.F_bound_L
            
            # We have to convert the boundary force to the robot frame
            self.F_bound = np.dot( self.bound_conversion, (self.F_bound_R - self.F_bound_L) )
        
        # Update robot position
        self.robot_prev_pose = self.robot_pose
        # else: 
        #     self.F_bound = np.array([0, 0])    
        
        # Reset to intial values of the related forces
        self.F_des = np.array([0.0, 0.0])
        self.F_soc = np.array([0.0, 0.0])
        
        if len(markers_msg.data) != 0:
            self.get_logger().info('Marker detected !!!')
            person_data = np.array(markers_msg.data)
            
            if self.prev_obj_position is None or self.prev_time is None:
                # Initial position setup
                self.prev_obj_position = [person_data[0], person_data[1]]
                self.prev_time = current_time
                return
        
            if self.prev_obj_position is not None and self.prev_time is not None:
                # Calculate displacement and time interval
                delta_x = person_data[0] - self.prev_obj_position[0]
                delta_y = person_data[1] - self.prev_obj_position[1]
                delta_t = (current_time - self.prev_time).nanoseconds / 1e9

                if delta_t <= 0:
                    self.get_logger().warn('Time interval must be positive')
                    return

                # Calculate velocity components
                self.object_linear_vel.x = delta_x / delta_t
                self.object_linear_vel.y = delta_y / delta_t
            
            self.prev_obj_position = [person_data[0], person_data[1]]
            self.prev_time = current_time
            
            # Compute the linear velocity of the robot relative to the object
            # TODO: This should be in robot frame, so there is no linear.y velocity
            vec_relative_vel = np.array([self.robot_velocity.linear.x - self.object_linear_vel.x, 0.0 - self.object_linear_vel.y]) # TODO: Checked
            
            # Compute the vectorized distance between the robot and the object
            # TODO: This should probably only be computed by the human position (the relative position to the robot), no need to refer to the origin
            vec_dis = np.array([-person_data[0], -person_data[1]]) # TODO: Checked
            
            # Encounter angle's cosine calculation
            cos_phi_ij = (vec_relative_vel / np.linalg.norm(vec_relative_vel)) * (-vec_dis / np.linalg.norm(vec_dis))
            if np.isnan(cos_phi_ij[0]):
                cos_phi_ij = np.array([0.0, 0.0])
            
            # Compute the object's anisotropic behavior
            Gamma = self.gamma_function(self.Lambda, cos_phi_ij)
            
            # Compute the distance between the robot and the object
            # TODO: This should probably only be computed by the human position (the relative position to the robot), no need to refer to the origin
            distance = math.sqrt((person_data[0])**2 + (person_data[1])**2) # TODO: Checked
                                            
            # Compute the social force applied to the mobile robot
            F_soc = Gamma * self.alpha * math.exp(-distance / self.beta) * (vec_dis / distance)

            self.F_soc = F_soc
            if math.isnan(self.F_soc[0]):
                self.F_soc = np.array([0.0, 0.0])
        
        else:
            self.get_logger().info('No marker detected !!!')
            self.F_soc = np.array([0.0, 0.0])
                
        # Current velocities of the mobile robot
        # TODO: This should be in robot frame, so there is no linear.y velocity
        self.v_cur = np.array([self.robot_velocity.linear.x, 0]) # TODO: Checked
                
        # Designed velocity of the mobile robot
        s_ = np.linalg.norm(self.F_soc) + np.linalg.norm(self.F_bound)
        if np.isnan(s_):
            s_ = 0.0
        # self.v_des = math.exp( -s_ / self.sigma ) * np.array([self.v_max_robot, 0])
        # To robot frame
        self.v_des = math.exp( -s_ / self.sigma ) * np.dot(self.des_conversion, np.array([self.v_max_robot, 0]))
        
        # Compute the designated force applied to the mobile robot
        # Robot frame
        F_des = self.total_mass * (self.v_des - self.v_cur) / self.tau_d
        # F_des = np.dot(self.des_conversion, F_des)
        self.F_des = F_des
            
        # Total force applied to the mobile robot
        F_total = self.F_des + self.F_soc + self.F_bound
        # To Robot frame
        # self.F_total = np.dot(self.total_conversion, F_total) 
        self.F_total = F_total
        
        print("=======================================")
        print("- Pose different: ", pose_diff)
        print("-----")
        print("- v_des: ", self.v_des)
        print("- F_des: ", self.F_des)
        print("-----")
        print("- F_soc: ", self.F_soc)
        print("-----")
        print("- Left_distance: ", d2bound_L if not self.using_lidar else self.left_distance)
        print("- Right_distance: ", d2bound_R if not self.using_lidar else self.right_distance)
        print("- F_bound: ", self.F_bound)
        print("-----")
        print("- Total force: ", self.F_total)
        print("\n")

        # Create and publish the force message
        force_msg = Float32MultiArray()
        force_msg.data.append(self.F_total[0])
        force_msg.data.append(self.F_total[1])
        self.force_publisher.publish(force_msg)
        # self.get_logger().info(f'Fm: {self.F_total[0]} | Fn: {self.F_total[1]}')
        
                            
    def update_dynamic(self, Fm, Fn, v_x, omega_z, theta, dt):
        # State space model of the pure dynamics
        # x_dot = v_x * np.cos(theta)
        # y_dot = v_x * np.sin(theta)
        theta_dot = omega_z
        z_in = np.array([v_x, omega_z])
        
        # Reduced Dynamic model
        # Compute Inertial matrix
        Mn = np.array([[self.total_mass, 0], [0, self.total_mass*self.cam2com + self.I_w + self.I_b]])
        # Compute Coriolis/Centrifugal matrix
        Cn = np.array([[0, -self.total_mass*self.cam2com*theta_dot], [self.total_mass*self.cam2com*theta_dot, 0]])
        # Compute Multiply matrix for Input's dynamic model
        Bn = np.array([[1, 0, 0], [0, self.cam2com, 1]])
        
        # Update Dynamic
        Z_dot = np.dot( np.linalg.inv(Mn), ( -np.dot( Cn, z_in ) + np.dot( Bn, np.array([Fm, Fn, 0]) ) ) )
        z_in += Z_dot * dt
        
        return z_in
        

def main(args=None):
    rclpy.init(args=args)
    updater = DynamicUpdater()

    # Create an instance of MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    # Add the node to the executor
    executor.add_node(updater)

    try:
        rclpy.spin(updater)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Stopping Dynamic Updater...")
    finally:
        # Clean up and shutdown
        executor.shutdown()
        updater.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
