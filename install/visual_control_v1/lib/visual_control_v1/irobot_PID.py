#!/usr/bin/env python3
from numba import cuda
import numpy as np
import math
import numba
import copy
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile, qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist, PoseStamped,Point, Pose2D
from sensor_msgs.msg import Imu
from std_msgs.msg import Int64, Float64


class LocalPlanningNode(Node):
    def __init__(self):
        super().__init__('irobot_PID')
        self.qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,depth=1)

        self.T = self.declare_parameter('T',5.0).get_parameter_value().double_value
        self.samplingtime = self.declare_parameter('samplingtime',0.01).get_parameter_value().double_value
        self.max_acc = self.declare_parameter('max_acc',3.0).get_parameter_value().double_value
        self.max_ang_acc = self.declare_parameter('max_ang_acc',10.0).get_parameter_value().double_value       
        self.lim_max_vel = self.declare_parameter('lim_max_vel',0.7).get_parameter_value().double_value
        self.lim_min_vel = self.declare_parameter('lim_min_vel',0.1).get_parameter_value().double_value
        self.lim_max_ang_vel = self.declare_parameter('lim_max_ang_vel',0.78539).get_parameter_value().double_value
        self.goal_tolerate_dis = self.declare_parameter('goal_tolerate_dis',0.5).get_parameter_value().double_value
        self.goal_tolerate_ang = self.declare_parameter('goal_tolerate_ang',0.5).get_parameter_value().double_value
        self.kp_dis = self.declare_parameter('kp_dis',1.0).get_parameter_value().double_value
        self.kp_ang = self.declare_parameter('kp_ang',1.5).get_parameter_value().double_value
        self.ki_ang = self.declare_parameter('ki_ang',0.015).get_parameter_value().double_value
        self.kd_ang = self.declare_parameter('kd_ang',0.05).get_parameter_value().double_value

        self.get_goal = False
        self.distance_check = True
        self.goal = [0.0, 0.0, 0.0]
        self.pre_goal = [0.0, 0.0, 0.0]
        self.error_sum = 0.0
        self.pre_error = 0.0
        self.pre_vel = [0.0, 0.0]
        self.count = 0
        self.count_stop=0

        self.sub = self.create_subscription(Pose2D, "goal_point", self.listener_callback_goal, self.qos)

        self.timer = self.create_timer(self.samplingtime, self.timer_callback)  # Timer for periodic execution

        self.pub_vel = self.create_publisher(Twist, "cmd_vel", 1)
        self.local_path_publisher = self.create_publisher(Path, '/local_path', 10)
        self.sampled_path_publisher = self.create_publisher(MarkerArray, '/sampled_path', 10)

    def listener_callback_goal(self, msg):
        self.goal[0] = msg.x
        self.goal[1] = msg.y
        self.goal[2] = msg.theta
        self.get_goal = True
        self.count_stop=0
        
    def PID_Control(self,distance,theta_error, kp, ki, kd, kp2):
    
        v = kp2 * distance
        # theta_robot_goal = math.atan2(self.goal[1], self.goal[0])
        self.error_sum = self.error_sum + theta_error
        w = kp * (theta_error) + ki * self.error_sum + kd * (theta_error - self.pre_error)
        self.pre_error = theta_error
        if self.error_sum > 1.0: self.error_sum = 0.0

        return v, w

    def set_range(self):
        range_ang_vel = self.samplingtime * self.max_ang_acc
        min_ang_vel = max(self.pre_vel[1] - range_ang_vel, - self.lim_max_ang_vel)
        max_ang_vel = min(self.pre_vel[1] + range_ang_vel, self.lim_max_ang_vel)

        range_vel = self.samplingtime * self.max_acc
        min_vel = max(self.pre_vel[0] - range_vel, self.lim_min_vel)
        max_vel = min(self.pre_vel[0] + range_vel, self.lim_max_vel)
        return min_vel,  max_vel, min_ang_vel, max_ang_vel

    def clamp(self, data, min, max):
        if data < min:
            return min
        elif data > max:
            return max
        else:
            return data

    def timer_callback(self):
        self.count_stop=self.count_stop+1
        print(self.count_stop)
        twist = Twist()
        self.current_useq = self.pre_vel

        if self.get_goal == True:
            distance = math.hypot(self.goal[0], self.goal[1])
            if distance < self.goal_tolerate_dis:
                self.distance_check = False

            if self.count_stop>30:
                v=0.0
                w=0.0
                self.error_sum=0.0
                self.distance_check=False
                self.get_goal=False
                print("stop")
            
            min_vel,  max_vel, min_ang_vel, max_ang_vel = self.set_range()

            v, w = self.PID_Control(distance,self.goal[2],self.kp_ang, self.ki_ang, self.kd_ang, self.kp_dis)
            
            v = self.clamp(v, min_vel, max_vel)
            w = self.clamp(w, min_ang_vel, max_ang_vel)


            
            if self.distance_check: twist.linear.x = float(v)
            twist.angular.z = float(w)

            self.pre_vel = np.array([v, w])

        elif self.get_goal == False:
            twist.linear.x = float(0.0)
            if self.pre_vel[1] > 0.0: twist.angular.z = float(0.2)
            else:  twist.angular.z = float(-0.2)
        print("anlge:",self.goal[2])
        print("get_goal = ", self.get_goal)
        print("distance_check = ", self.distance_check)
        print(f'v = {twist.linear.x}, w = {twist.angular.z}')
        print("===================================================")

        self.pub_vel.publish(twist)
        self.distance_check = True

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

