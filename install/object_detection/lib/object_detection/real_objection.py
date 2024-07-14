#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from time import sleep
from image_geometry import PinholeCameraModel
import struct
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray
import numpy as np

class RealObjectionNode(Node):

    def __init__(self):
        """
        Initialize the node, open serial port
        """
        # Init node
        super().__init__('real_objection')

        # parameters
        self.declare_parameter('cam_name', 'cam_name')
        self.cam_name = self.get_parameter('cam_name').get_parameter_value().string_value

        self.declare_parameter('topic_camera_depth', 'topic_camera_depth')
        self.topic_camera_depth = self.get_parameter('topic_camera_depth').get_parameter_value().string_value

        self.declare_parameter('topic_camera_info', 'topic_camera_info')
        self.topic_camera_info = self.get_parameter('topic_camera_info').get_parameter_value().string_value

        self.declare_parameter('base_frame', 'base_frame')
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self.declare_parameter('topic_person', 'topic_person')
        self.topic_person = self.get_parameter('topic_person').get_parameter_value().string_value

        self.declare_parameter('topic_object', 'topic_object')
        self.topic_object = self.get_parameter('topic_object').get_parameter_value().string_value

        self.declare_parameter('permit_distance', 3.0)
        self.permit_distance = self.get_parameter('permit_distance').get_parameter_value().double_value

        self.declare_parameter('limit_x', 260.0)
        self.limit_x = self.get_parameter('limit_x').get_parameter_value().double_value

        self.declare_parameter('limit_y', 300.0)
        self.limit_y = self.get_parameter('limit_y').get_parameter_value().double_value

        # Create a topic for the array of markers
        self.topic_person_array = self.topic_person + "_array"
        self.need_info = True
        self.camera = PinholeCameraModel()
        self.count_maker = 0
        self.read_depth = False
        self.read_object = False

        # Publisher for MarkerArray
        self.pub_maker = self.create_publisher(MarkerArray, self.topic_person, 10)

        # Subscriptions to camera depth, camera info, and object position
        self.subscription_camera_depth = self.create_subscription(
            Image,
            self.topic_camera_depth,
            self.listener_callback_image_depth,
            1)

        self.subscription_camera_info = self.create_subscription(
            CameraInfo,
            self.topic_camera_info,
            self.listener_callback_image_info,
            1)

        self.subscription_object = self.create_subscription(
            Int16MultiArray,
            self.topic_object,
            self.listener_callback_object,
            2)

        # Data structure for storing object position
        self.data_object = Float32MultiArray()

        # Publisher for the array of object positions
        self.pub_object = self.create_publisher(Float32MultiArray, self.topic_person_array, 2)

        # Timer callback for processing data
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def get_distance(self, data, u, v):
        Idx = u + data.width * v
        depths = struct.unpack('f' * (len(data.data) // 4), data.data)
        return depths[Idx]

    def pixel_point(self, data, position):
        # print(position)
        list_real_position = []
        num_object = int(len(position) / 2)
        u = position[:num_object]
        v = position[num_object:]
        for i in range(num_object):
            distance = self.get_distance(data, u[i], v[i])
            if 0.0 < distance < self.permit_distance:
                xyz_ray = self.camera.projectPixelTo3dRay([u[i], v[i]])
                x = xyz_ray[2] * distance
                y = xyz_ray[0] * distance
                z = xyz_ray[1] * distance
                point = [x, -y, z]
                list_real_position.append(point)
        return list_real_position

    def listener_callback_image_depth(self, data):
        self.depth = data
        self.read_depth = True

    def listener_callback_image_info(self, data):
        if self.need_info:
            self.get_logger().info('Getting camera information')
            self.camera.fromCameraInfo(data)
            self.need_info = False

    def listener_callback_object(self, data):
        self.position = data.data
        # print('position = ', self.position)
        # print('-------------------')
        self.read_object = True

    def publish_maker(self, position):
        markerArray = MarkerArray()
        num_object = len(position)
        for i in range(num_object):
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.id = i
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 1.0
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = float(position[i][0])
            marker.pose.position.y = float(position[i][1])
            marker.pose.position.z = 0.5
            marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            markerArray.markers.append(marker)
        if num_object < self.count_maker:  # delete older marker
            for i in range(num_object, self.count_maker):
                marker = Marker()
                marker.header.frame_id = self.base_frame
                marker.type = marker.CYLINDER
                marker.action = marker.DELETE
                marker.id = i
                markerArray.markers.append(marker)
        self.count_maker = num_object
        self.pub_maker.publish(markerArray)

    def publish_object(self, position):
        self.data_object.data = position
        self.pub_object.publish(self.data_object)

    def timer_callback(self):
        # print(self.need_info , self.read_depth, self.read_object)
        if self.need_info == False and self.read_depth == True and self.read_object == True:
            # print("in")
            position = self.pixel_point(self.depth, self.position)
            # print(position)
            # Convert to one list
            merged_list = [item for sublist in position for item in sublist]
            # print(merged_list)
            self.publish_maker(position)
            self.publish_object(merged_list)
            self.read_depth = False
            self.read_object = False

def start():
    rclpy.init()
    real_object = RealObjectionNode()
    try:
        rclpy.spin(real_object)
        real_object.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Closing real objection...")

if __name__ == '__main__':
    start()
