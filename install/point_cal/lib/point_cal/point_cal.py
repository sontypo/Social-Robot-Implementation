#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from time import sleep
from image_geometry import PinholeCameraModel
import struct
from std_msgs.msg import Int16MultiArray, Float32MultiArray
from nav_msgs.msg import Path
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, Pose2D, Pose
from tf2_ros.buffer import Buffer
import tf2_ros
from tf2_geometry_msgs import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

class ref_calculate(Node):

    def __init__(self) :
        """
        Initialize the node, open serial port
        """        
        # Init node
        super().__init__('point_cal')

        self.qos = QoSProfile(depth=1)
        self.qos_scan = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,depth=1)

        # parameters
        self.declare_parameter('radius', 1.5)
        self.radius = self.get_parameter('radius').get_parameter_value().double_value

        self.declare_parameter('circle_num', 51)
        self.circle_num = self.get_parameter('circle_num').get_parameter_value().integer_value

        self.declare_parameter('laser_max_dis', 5.0)
        self.laser_max_dis = self.get_parameter('laser_max_dis').get_parameter_value().double_value

        self.declare_parameter('coff_static_obstacle', 0.5)
        self.coff_static_obstacle = self.get_parameter('coff_static_obstacle').get_parameter_value().double_value

        self.declare_parameter('obstacle_safe_dis', 0.5)
        self.obstacle_safe_dis = self.get_parameter('obstacle_safe_dis').get_parameter_value().double_value

        self.declare_parameter('coff_robot_dis', 1.0)
        self.coff_robot_dis = self.get_parameter('coff_robot_dis').get_parameter_value().double_value

        self.declare_parameter('coff_robot_ang', 1.0)
        self.coff_robot_ang = self.get_parameter('coff_robot_ang').get_parameter_value().double_value

        self.declare_parameter('topic_person', 'person_array')
        self.topic_person = self.get_parameter('topic_person').get_parameter_value().string_value

        self.declare_parameter('base_link_frame', 'base_link')
        self.base_link_frame = self.get_parameter('base_link_frame').get_parameter_value().string_value
       
        self.declare_parameter('camera_frame', 'camera_link')
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

        self.declare_parameter('base_scan_frame', 'base_scan')
        self.base_scan_frame = self.get_parameter('base_scan_frame').get_parameter_value().string_value

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.person = []
        self.obstacle = []
        self.Point = np.zeros((self.circle_num, 4)) #[x, y, angle from robot, cost]
        self.cost = np.zeros((self.circle_num, 3)) #[total, angle, obs]
        self.goal_pos = Pose2D()

        self.subscription_camera = self.create_subscription(
            Float32MultiArray,
            self.topic_person,
            self.listener_callback_object,
            1)
        
        self.sub = self.create_subscription(LaserScan,
                                            '/scan',
                                            self.scan_cb,
                                            self.qos_scan)
        
        # Publisher for MarkerArray
        self.pub_maker = self.create_publisher(MarkerArray, 'predect_circle', 10)
        self.goal_pub = self.create_publisher(Pose2D, 'goal_point', self.qos)
        self.oval_pub = self.create_publisher(MarkerArray, "oval_point", self.qos)

        
    def listener_callback_object(self, msg):
        point = self.send_tf(msg, self.camera_frame, self.base_link_frame)
        if len(point) == 2:
            self.person = [round(point[0], 4), round(point[1], 4)]

            self.find_point()

    def send_tf(self,msg, before_frame, after_frame):
        point = []
        x_data = msg.data[0::3]
        y_data = msg.data[1::3] 
        num_object = len(x_data)
        for i in range(num_object):
            point_stamped = PoseStamped()
            point_stamped.pose.position.x = x_data[i]
            point_stamped.pose.position.y = y_data[i]
            point_stamped.header.frame_id = before_frame
            try:
                t = self.tf_buffer.transform(point_stamped, after_frame)
                point = [t.pose.position.x, t.pose.position.y]
            except:
                print('error')
                pass    
        return point

    def check_angle(self, theta:float):
        while(abs(theta) > math.pi):
            theta = theta - self.check_sign(theta)*math.pi
        return theta
    
    def check_sign(self,n:float):
        return -1 if n < 0 else 1
    
    def transform_laser(self, laser_msg:LaserScan):
        min_ang = laser_msg.angle_min
        increment = laser_msg.angle_increment
        self.obstacle = []
        for i, value in enumerate(laser_msg.ranges):
            angle = self.check_angle(i * increment + min_ang)
            if value < self.laser_max_dis:
                x = math.cos(angle) * value
                y = math.sin(angle) * value
                point_stamped = PoseStamped()
                point_stamped.pose.position.x = float(x)
                point_stamped.pose.position.y = float(y)
                point_stamped.pose.position.z = 1.0
                point_stamped.header.frame_id = self.base_scan_frame
                try:
                    t = self.tf_buffer.transform(point_stamped, self.base_link_frame)
                    point_new = Point()
                    point_new.x = t.pose.position.x
                    point_new.y = t.pose.position.y
                    point_new.z = t.pose.position.z
                    self.obstacle.append([point_new.x,point_new.y])
                    if self.get_laser == False:
                        self.get_laser = True
                except:
                    # print('transform error!')
                    pass 

    def scan_cb(self, msg:LaserScan):
        self.transform_laser(msg)

    def create_point(self):
        th =  math.atan2(self.person[1], self.person[0])
        theta = np.linspace(th + math.pi/2, th + math.pi*3/2, self.circle_num)

        for i in range(self.circle_num):
            self.Point[i, 0] = self.person[0] + self.radius * np.cos(theta[i])
            self.Point[i, 1] = self.person[1] + self.radius * np.sin(theta[i])
            self.Point[i, 2] = math.atan2(self.Point[i,1], self.Point[i,0])
    
    def ang_cost(self, x:float, y:float):
        # print(x,y)
        th_person = math.atan2(self.person[1], self.person[0])
        th_point = math.atan2(y, x)
        th_point_to_person = math.atan2(self.person[1]-y, self.person[0]-x)
        # print(abs(th_person - th_point_to_person))
        return abs(th_person - th_point_to_person) / np.pi *100
    
    def dis_cost(self, x:float, y:float):
        return math.hypot(x,y)
        
    def obstacle_cost(self, x:float, y:float):
        cost = 0.0
        for obs in self.obstacle:
            dis = math.hypot(x-obs[0],y-obs[1])
            if dis <= self.obstacle_safe_dis:
                # cost += 1e10
                return 1e15
            elif dis <= 2*self.obstacle_safe_dis:
                tmp = (2*self.obstacle_safe_dis-dis) /  self.obstacle_safe_dis * 100
                if tmp > cost:
                    cost = tmp
        return cost
    
    def cal_cost(self, x:float, y:float):
        cost = 0.0
        cost_obs = 0.0
        cost_ang = 0.0

        cost_ang = self.coff_robot_ang * self.ang_cost(x,y)

        if self.obstacle != []:
            cost_obs = self.coff_static_obstacle * self.obstacle_cost(x,y)
        
        cost = cost_ang + cost_obs
        return cost, cost_ang, cost_obs

    def find_point(self):
        self.create_point()
        for i in range(self.circle_num):
            cost, cost_ang, cost_obs= self.cal_cost(self.Point[i,0], self.Point[i,1])
            self.Point[i,3]  = cost
            self.cost[i,0]  = cost
            self.cost[i,1]  = cost_ang
            self.cost[i,2]  = cost_obs
        min = np.argmin(self.cost, axis=0)
        self.goal_pos.x = self.Point[min[0],0] 
        self.goal_pos.y = self.Point[min[0],1]
        self.goal_pos.theta = math.atan2(self.person[1], self.person[0])
        self.goal_pub.publish(self.goal_pos)
        #print(self.goal_pos.x, self.goal_pos.y)
        self.pub_oval(min[0], min[1])
    
    def pub_oval(self, min:int, min_ang:int):
        marker_array = MarkerArray()
        for i in range(self.circle_num):
            marker = Marker()
            marker.header.frame_id = self.base_link_frame
            marker.ns = 'oval_point'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            if self.Point[i,3] >= 1e10:
                marker.color.b = 1.0
            elif i != min:
                marker.color.g = 1.0
            if i == min_ang:
                marker.color.r = 0.5
            pose = Pose()
            pose.position.x = self.Point[i,0]
            pose.position.y = self.Point[i,1]
            marker.pose = pose
            marker_array.markers.append(marker)

        self.oval_pub.publish(marker_array)


def start():
    rclpy.init()
    ref = ref_calculate()
    try:
        rclpy.spin(ref)
        ref.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Closing ref calculate...")
    
   

if __name__ == '__main__':
    start()
