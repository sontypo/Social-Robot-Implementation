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

class DynamicUpdater(Node):
    def __init__(self):
        super().__init__('dynamic_updater')
        self.robot_pose = Point()
        self.robot_velocity = Twist()
        
        self.total_mass = 25.0
        m_base = 20.0
        m_wheel = 2.5
        R_wheel = 0.08
        self.I_w = 1/2 * m_wheel * R_wheel*R_wheel
        self.I_b = 1/12 * m_base * (0.26*0.26 + 0.13*0.13 + 0.605*0.605)  # 0.416 + 0.8
        self.d2origin = 0.08 #m
        self.tau_d = 4.0
        self.v_max_robot = 2.5
        # designated velocity 
        self.sigma = 3.0    # for computing new v_des
        # self.v_des = np.array([self.v_max_robot, 0])      # this one is for the previous version
        self.v_des = np.array([0, 0])   # new version of v_des for computing F_des
        self.F_des = np.array([0, 0])
        self.F_soc = np.array([0, 0])
        self.F_total = np.array([0, 0])
        self.theta = 0.0
        
        # Force manitude
        self.alpha = 12.0
        # Force range
        self.beta = 0.66
        
        # Added boundary force's terms
        self.F_bound_R = None
        self.F_bound_L = None
        self.F_bound = np.array([0, 0])
        self.boundary_dis = 2.5
        self.alpha_bound = 15
        self.beta_bound = 0.25

        # Subscriber for the robot's position 
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Subscriber for the object's position (topic /person in this case)
        self.create_subscription(MarkerArray, '/person', self.object_position_callback, 10)
        
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
        self.diff_thres = 0.5 #m
        
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

    def object_position_callback(self, markers_msg):
        if self.robot_pose is None:
            self.get_logger().warn('Waiting for odometry callback...')
            return
        
        if self.robot_prev_pose is None:
            self.robot_prev_pose = self.robot_pose
            return
        
        pose_diff = math.sqrt((self.robot_prev_pose.x - self.robot_pose.x)**2 + (self.robot_prev_pose.y - self.robot_pose.y)**2)
        # Compute the boundary forces applied to the mobile robot
        if pose_diff >= self.diff_thres:
            ''' Assume that the mobile robot is always tends to be positioned at the center of the hallway, which is the intial position of it.
                The boundary forces applied to the robot are from both sides, the distance between it and the wall is determined as: d2bound_L & d2bound_R '''
            d2bound_L = (self.boundary_dis / 2) - self.robot_pose.y
            d2bound_R = (self.boundary_dis / 2) - (-self.robot_pose.y)
            # Compute the boundary forces from both sides
            self.F_bound_L = np.array( [self.alpha_bound * math.exp(-d2bound_L / self.beta_bound) * math.sin(self.theta), \
                                        self.alpha_bound * math.exp(-d2bound_L / self.beta_bound) * math.cos(self.theta)] )
            self.F_bound_R = np.array( [self.alpha_bound * math.exp(-d2bound_R / self.beta_bound) * math.sin(self.theta), \
                                        self.alpha_bound * math.exp(-d2bound_R / self.beta_bound) * math.cos(self.theta)] )
            # Total boundary force
            self.F_bound = self.F_bound_R - self.F_bound_L
            # Update robot position
            self.robot_prev_pose = self.robot_pose
        else: 
            self.F_bound = np.array([0, 0])
            
        # Designed velocity of the mobile robot
        s_ = np.linalg.norm(self.F_des) + np.linalg.norm(self.F_bound)
        self.v_des = math.exp( -s_ / self.sigma ) * self.v_max_robot
        
        # Current velocities of the mobile robot
        self.v_cur = np.array([self.robot_velocity.linear.x, self.robot_velocity.linear.y])
        
        # Compute the designated force applied to the mobile robot
        F_des = self.total_mass * (self.v_des - self.v_cur) / self.tau_d
        self.F_des = F_des
        print(self.v_des)
        
        for marker in markers_msg.markers:
            if marker is not None:
                self.get_logger().info('Marker detected !!!')
                # Compute the vectorized distance between the robot and the object
                vec_dis = np.array([-marker.pose.position.x + self.robot_pose.x, -marker.pose.position.y + self.robot_pose.y])
                
                # Compute the distance between the robot and the object
                distance = math.sqrt((marker.pose.position.x - self.robot_pose.x)**2 + (marker.pose.position.y - self.robot_pose.y)**2)
                                              
                # Compute the social force applied to the mobile robot
                self.F_soc = self.alpha * math.exp(-distance / self.beta) * (vec_dis / distance) 
                
            # Total force applied to the mobile robot
            self.F_total = F_des + self.F_soc + self.F_bound

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
