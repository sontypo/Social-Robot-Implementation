import rclpy
import rclpy.executors
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist
from std_msgs.msg import Int16MultiArray, Float32MultiArray
from visualization_msgs.msg import MarkerArray
from gazebo_msgs.msg import ModelStates
import math
import numpy as np
from tf_transformations import euler_from_quaternion
import time

class DynamicUpdater(Node):
    def __init__(self):
        super().__init__('dynamic_updater') # type: ignore
        self.robot_pose = Point()
        self.robot_velocity = Twist()
        self.object_linear_vel = Twist().linear
        
        self.total_mass = 25.0
        m_base = 20.0
        m_wheel = 2.5
        R_wheel = 0.08
        self.I_w = 1/2 * m_wheel * R_wheel*R_wheel
        # self.I_b = 1/12 * m_base * (0.26*0.26 + 0.13*0.13 + 0.605*0.605)  # 0.416 + 0.8
        self.I_b = 1.216
        self.d2origin = 0.2 #m
        self.tau_d = 2.953125    # original: 0.2953125 - it was Ali's params
        self.v_max_robot = 2.5
        # designated velocity 
        self.sigma = 38.8623    # for computing new v_des
        # self.v_des = np.array([self.v_max_robot, 0])      # this one is for the previous version
        self.v_des = np.array([0, 0])   # new version of v_des for computing F_des
        self.F_des = np.array([0, 0])
        self.F_soc = np.array([0, 0])
        self.F_total = np.array([0, 0])
        self.theta = 0.0
        self.Lambda = 0.04375
        
        # Force manitude
        self.alpha = 83.518066
        # Force range
        self.beta = 1.806738
        
        # Added boundary force's terms
        self.F_bound = np.array([0, 0])
        self.boundary_dis = 2.5
        self.alpha_bound = 100
        self.beta_bound = 3.5

        # Subscriber for the robot's position 
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Subscriber for the object's position (topic /person in this case)
        self.create_subscription(MarkerArray, '/person', self.object_position_callback, 10)
        self.object_positions = {}  # Store the previous positions and timestamps of object

        
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
        
    def get_object_linear_vel(self, prev_pos, cur_pos, time_diff):
        linear_velocity = Twist().linear
        linear_velocity.x = (cur_pos.x - prev_pos.x) / time_diff
        linear_velocity.y = (cur_pos.y - prev_pos.y) / time_diff
        linear_velocity.z = (cur_pos.z - prev_pos.z) / time_diff
        return linear_velocity
    
    # The object's anisotropic behavior function
    def gamma_function(self, Lambda, cos_phi_ij):
        return Lambda + 0.5 * (1 - Lambda) * (1 + cos_phi_ij)

    def object_position_callback(self, markers_msg):
        current_time = time.time()
        if self.robot_pose is None:
            self.get_logger().warn('Waiting for odometry callback...')
            return
        
        if self.robot_prev_pose is None:
            self.robot_prev_pose = self.robot_pose
            return
        
        pose_diff = math.sqrt((self.robot_prev_pose.x - self.robot_pose.x)**2 + (self.robot_prev_pose.y - self.robot_pose.y)**2)
        # Compute the boundary force applied to the mobile robot
        if pose_diff >= self.diff_thres:
            ''' Assume that the mobile robot is always tends to be positioned at the center of the hallway, which is the intial position of it.
                The boundary force applied to the robot depends on the changes in position of the robot along the y-axis '''
            # The boundary force
            # theta in F_bound's term is zero due to the fact that boundary forces are only affected along the y-axis
            if self.robot_pose.y > 0:
                self.F_bound = - np.array( [self.alpha_bound * math.exp(- self.robot_pose.y / self.beta_bound) * math.sin(0), \
                                        self.alpha_bound * math.exp(- self.robot_pose.y / self.beta_bound) * math.cos(0)] )
            elif self.robot_pose.y < 0:
                self.F_bound = np.array( [self.alpha_bound * math.exp(- abs(self.robot_pose.y) / self.beta_bound) * math.sin(0), \
                                        self.alpha_bound * math.exp(- self.robot_pose.y / self.beta_bound) * math.cos(0)] )
            # Update robot position
            self.robot_prev_pose = self.robot_pose
        else: 
            self.F_bound = np.array([0, 0])    
        
        print("--------------------------------")
        print("Pose different: ", pose_diff)
        print("F_bound: ", self.F_bound)
        
        for marker in markers_msg.markers:
            object_id = marker.id
            current_position = marker.pose.position

            if object_id in self.object_positions:
                previous_position, previous_time = self.object_positions[object_id]

                time_diff = current_time - previous_time
                if time_diff > 0.0:
                    # Calculate object linear velocity
                    self.object_linear_vel = self.get_object_linear_vel(previous_position, current_position, time_diff)

            self.object_positions[object_id] = (current_position, current_time)
            
            # Reset to intial values of the related forces
            self.F_des = np.array([0, 0])
            self.F_soc = np.array([0, 0])
            
            if marker is not None:
                self.get_logger().info('Marker detected !!!')
                # Compute the linear velocity of the robot relative to the object
                vec_relative_vel = np.array([self.robot_velocity.linear.x - self.object_linear_vel.x, self.robot_velocity.linear.y - self.object_linear_vel.y])
                
                # Compute the vectorized distance between the robot and the object
                vec_dis = np.array([-marker.pose.position.x + self.robot_pose.x, -marker.pose.position.y + self.robot_pose.y])
                
                # Encounter angle's cosine calculation
                cos_phi_ij = (vec_relative_vel / np.linalg.norm(vec_relative_vel)) * (-vec_dis / np.linalg.norm(vec_dis))
                
                # Compute the object's anisotropic behavior
                Gamma = self.gamma_function(self.Lambda, cos_phi_ij)
                
                # Compute the distance between the robot and the object
                distance = math.sqrt((marker.pose.position.x - self.robot_pose.x)**2 + (marker.pose.position.y - self.robot_pose.y)**2)
                                              
                # Compute the social force applied to the mobile robot
                self.F_soc = Gamma * self.alpha * math.exp(-distance / self.beta) * (vec_dis / distance) 
                
        # Current velocities of the mobile robot
        self.v_cur = np.array([self.robot_velocity.linear.x, self.robot_velocity.linear.y])
                
        # Designed velocity of the mobile robot
        s_ = np.linalg.norm(self.F_soc) + np.linalg.norm(self.F_bound)
        self.v_des = math.exp( -s_ / self.sigma ) * np.array([self.v_max_robot, 0])
        
        # Compute the designated force applied to the mobile robot
        F_des = self.total_mass * (self.v_des - self.v_cur) / self.tau_d
        self.F_des = F_des
        print("v_des: ", self.v_des)
        print("F_des: ", self.F_des)
        print("F_soc: ", self.F_soc)
            
        # Total force applied to the mobile robot
        self.F_total = self.F_des + self.F_soc + self.F_bound

        # Create and publish the force message
        force_msg = Float32MultiArray()
        force_msg.data.append(self.F_total[0])
        force_msg.data.append(self.F_total[1])
        self.force_publisher.publish(force_msg)
        # self.get_logger().info(f'Fm: {self.F_total[0]} | Fn: {self.F_total[1]}')
                            
            
    # def actor_pose_callback(self, actor):
    #     try:
    #         index = actor.name.index(self.actor_name)
    #         position = actor.pose[index].position

    #         actor_pose = Point()
    #         actor_pose.x = position.x
    #         actor_pose.y = position.y

    #         # Compute the vectorized distance between the robot and the object
    #         vec_dis = np.array([actor_pose.x - self.robot_pose.x, actor_pose.y - self.robot_pose.y])
    #         # Compute the distance between the robot and the object
    #         distance = math.sqrt((actor_pose.x - self.robot_pose.x)**2 + (actor_pose.y - self.robot_pose.y)**2)
            
    #         # Current velocities of the mobile robot
    #         self.v_cur = np.array([self.robot_velocity.linear.x, self.robot_velocity.linear.y])
            
    #         # Compute the designated force applied to the mobile robot
    #         F_des = self.total_mass * (self.v_des - self.v_cur) / self.tau_d
            
    #         # Compute the social force applied to the mobile robot
    #         F_soc = self.alpha * math.exp(-distance / self.beta) * (vec_dis / distance) 
            
    #         # Total force applied to the mobile robot
    #         self.F_total = F_des + F_soc

    #         # # Create and publish the force message
    #         # force_msg = Int16MultiArray()
    #         # force_msg.data.append(F_total[0])
    #         # force_msg.data.append(F_total[1])
    #         # self.force_publisher.publish(force_msg)
    #         self.get_logger().info(f'Fm: {self.F_total[0]} | Fn: {self.F_total[1]}')
    #         print("Fn: {self.F_total[0]} | Fn: {self.F_total[1]}")
            
    #     except ValueError:
    #         self.get_logger().warn(f'{self.actor_name} not found in model states.')
        
    def update_dynamic(self, Fm, Fn, v_x, omega_z, theta, dt):
        # State space model of the pure dynamics
        # x_dot = v_x * np.cos(theta)
        # y_dot = v_x * np.sin(theta)
        theta_dot = omega_z
        z_in = np.array([v_x, omega_z])
        
        # Reduced Dynamic model
        # Compute Inertial matrix
        Mn = np.array([[self.total_mass, 0], [0, self.total_mass*self.d2origin + self.I_w + self.I_b]])
        # Compute Coriolis/Centrifugal matrix
        Cn = np.array([[0, -self.total_mass*self.d2origin*theta_dot], [self.total_mass*self.d2origin*theta_dot, 0]])
        # Compute Multiply matrix for Input's dynamic model
        Bn = np.array([[1, 0, 0], [0, self.d2origin, 1]])
        
        # Update Dynamic
        Z_dot = np.dot( np.linalg.inv(Mn), ( -np.dot( Cn, z_in ) + np.dot( Bn, np.array([Fm, Fn, 0]) ) ) )
        z_in += Z_dot * dt
        
        return z_in
        

def main(args=None):
    rclpy.init(args=args)
    updater = DynamicUpdater()
    try:
        rclpy.spin(updater)
        updater.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Stopping Dynamic Updater...")

if __name__ == '__main__':
    main()
