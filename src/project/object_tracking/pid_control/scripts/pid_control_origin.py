#!/usr/bin/env python3
import math, rclpy, time
import numpy as np
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile, qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist, PoseStamped,Point, Pose2D
from sensor_msgs.msg import Imu
from std_msgs.msg import Int64, Float64


class LocalPlanningNode(Node):
    def __init__(self):
        super().__init__('pid_control')
        self.qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,depth=1)

        self.Kp = self.declare_parameter('Kp',5.0).get_parameter_value().double_value
        self.Ki = self.declare_parameter('Ki',5.0).get_parameter_value().double_value
        self.Kd = self.declare_parameter('Kd',5.0).get_parameter_value().double_value
        self.samplingtime = self.declare_parameter('samplingtime',1.0).get_parameter_value().double_value     
        self.max_lin_vel = self.declare_parameter('max_lin_vel',0.7).get_parameter_value().double_value
        self.max_ang_vel = self.declare_parameter('max_ang_vel',0.78539).get_parameter_value().double_value
        self.goal_tolerate_dis = self.declare_parameter('goal_tolerate_dis',0.5).get_parameter_value().double_value
        self.goal_tolerate_ang = self.declare_parameter('goal_tolerate_ang',0.5).get_parameter_value().double_value

        self.get_goal = False
        self.distance_check = True
        self.angular_check = True
        self.goal = Pose2D()
        self.pre_goal = Pose2D()
        self.control = Twist()
        self.count = 0
        self.last_time = time.time()
        self.ITerm = np.zeros((3,1))

        self.sub = self.create_subscription(Pose2D, "goal_point", self.listener_callback_goal, self.qos)
        self.pub_vel = self.create_publisher(Twist, "cmd_vel", 1)

        self.timer = self.create_timer(self.samplingtime, self.timer_callback)  # Timer for periodic execution

    def listener_callback_goal(self, msg:Pose2D):
        self.goal = msg
        # self.goal.theta = math.atan2(self.goal.y, self.goal.x)
        self.get_goal = True
    
    def mul_pose2d(self, a:Pose2D, b:Pose2D):
        pose = Pose2D()
        pose.x = a.x - b.x
        pose.y = a.y - b.y
        pose.theta = a.theta - b.theta

        return pose
        
    def PID_Control(self):
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = self.mul_pose2d(self.goal, self.pre_goal)

        # calculate vx
        PTerm = self.Kp * self.goal.x # 比例
        self.ITerm[0] += self.goal.x  * delta_time # 积分
        if delta_time > 0:
            DTerm = delta_error.x / delta_time # 微分
        
        vx = PTerm + (self.Ki * self.ITerm[0]) + (self.Kd * DTerm)
        
        # calculate vy
        PTerm = self.Kp * self.goal.y # 比例
        self.ITerm[1] += self.goal.y  * delta_time # 积分
        if delta_time > 0:
            DTerm = delta_error.y / delta_time # 微分
        
        vy = PTerm + (self.Ki * self.ITerm[1]) + (self.Kd * DTerm)
        
        # calculate w(omega)
        PTerm = self.Kp * self.goal.theta # 比例
        self.ITerm[2] += self.goal.theta  * delta_time # 积分
        if delta_time > 0:
            DTerm = delta_error.theta / delta_time # 微分
        
        w = PTerm + (self.Ki * self.ITerm[2]) + (self.Kd * DTerm)

        self.last_time = self.current_time

        return vx, vy, w

    def clip(self, value, min, max):
        return min if value < min else max if value > max else value

    def check(self):
        self.count += 1

        if self.count % 50 == 0 and self.pre_goal == self.goal:
            self.get_goal = False
        if self.count % 200 == 0:
            self.ITerm[0] = 0.0
            self.ITerm[1] = 0.0
        self.pre_goal = self.goal

    def timer_callback(self):
        self.check()

        if self.get_goal == True:
            distance = math.hypot(self.goal.x, self.goal.y)
            if distance < self.goal_tolerate_dis:
                self.distance_check = False
            if abs(self.goal.theta) < self.goal_tolerate_ang:
                self.angular_check = False

            vx, vy, w = self.PID_Control()
            
            vx = self.clip(vx, -self.max_lin_vel, self.max_lin_vel)
            vy = self.clip(vy, -self.max_lin_vel, self.max_lin_vel)
            w = self.clip(w, -self.max_ang_vel, self.max_ang_vel)
            
            if self.distance_check: 
                self.control.linear.x = float(vx)
                self.control.linear.y = float(vy)
            if self.angular_check: self.control.angular.z = float(w)

        elif self.get_goal == False:
            if self.control.linear.x  > 0.0: self.control.angular.z = 0.1
            else:  self.control.angular.z = -0.1
            self.control.linear.x = 0.0
            self.control.linear.y = 0.0

        print("get_goal: ", self.get_goal)
        print("distance_check: ", self.distance_check)
        print("angular_check: ", self.angular_check)
        print(f'vx = {self.control.linear.x}, vy = {self.control.linear.y}, w = {self.control.angular.z}')
        print("===================================================")

        self.pub_vel.publish(self.control)
        self.distance_check = True
        self.angular_check = True

        # self.get_goal = False
        

def start():

    rclpy.init()
    planning = LocalPlanningNode()
    try:
        rclpy.spin(planning)
        planning.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Closing local_planning...")

if __name__ == '__main__':
    start()

