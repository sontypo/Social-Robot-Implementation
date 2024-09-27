#!/usr/bin/env python3

import rclpy
import rclpy.duration
import rclpy.executors
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import numpy as np
import  time, os
import struct
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray
import torch
import cv2
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from ultralytics import YOLO
from threading import Lock, Thread
from time import sleep
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray, Marker
# torch.cuda.set_device(0) # Set to your desired GPU number
class HumanDetection(Node):

    def __init__(self) :
        """
        Initialize the node, open serial port
        """        
        # Init node
        super().__init__('HumanDetection')
        # parameters
        self.declare_parameter('cam_name','cam_name')
        self.cam_name = self.get_parameter('cam_name').get_parameter_value().string_value

        self.declare_parameter('topic_camera','topic_camera')
        self.topic_camera = self.get_parameter('topic_camera').get_parameter_value().string_value

        self.declare_parameter('model_yolo','model_yolo')
        self.model_yolo = self.get_parameter('model_yolo').get_parameter_value().string_value

        self.declare_parameter('model_yolo_custom','model_yolo_custom')
        self.model_yolo_custom = self.get_parameter('model_yolo_custom').get_parameter_value().string_value

        self.my_cls = self.declare_parameter('my_cls',0).get_parameter_value().integer_value
        
        self.declare_parameter('topic_camera_depth', 'topic_camera_depth')
        self.topic_camera_depth = self.get_parameter('topic_camera_depth').get_parameter_value().string_value

        self.declare_parameter('topic_camera_info', 'topic_camera_info')
        self.topic_camera_info = self.get_parameter('topic_camera_info').get_parameter_value().string_value

        self.declare_parameter('base_frame', 'base_frame')
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self.declare_parameter('topic_person', 'topic_person')
        self.topic_person = self.get_parameter('topic_person').get_parameter_value().string_value

        self.declare_parameter('permit_distance', 4.0)
        self.permit_distance = self.get_parameter('permit_distance').get_parameter_value().double_value

        self.data_object=Int16MultiArray()

        self.bridge = CvBridge()
        self.dirPath = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
        self.weights_yolo = "/home/saun/ctrl_mr_dynamic/src/human_detection_model/weights/" + self.model_yolo
        self.weights_custom = "/home/saun/ctrl_mr_dynamic/src/human_detection_model/weights/" + self.model_yolo_custom
        self.lock = Lock()
        self.exit_signal=False
        self.run_signal=False
        
        # Create a topic for the array of markers
        self.topic_person_array = self.topic_person + "_pose_array"
        self.need_info = True
        self.camera_model = PinholeCameraModel()
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


        # Data structure for storing object position
        self.data_object = Float32MultiArray()

        # Publisher for the array of object positions
        self.person_pub = self.create_publisher(PoseArray, self.topic_person_array, 2)

        # Timer callback for processing data
        timer_period = 0.001  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription_camera = self.create_subscription(
            Image,
            self.topic_camera,
            self.listener_callback_image,
            1)
        
        
    def torch_thread(self,weights_yolo, weights_custom,conf_thres=0.6,iou_thres=0.45):
        print("Intializing Network...")
        # self.model = YOLO(weights_yolo)
        self.model = YOLO(weights_custom)
        while not self.exit_signal:
            try:
                if self.run_signal:
                    object_position = []
                    self.lock.acquire()
                    self.results=self.model(self.cv_image,verbose=False, conf=conf_thres,iou=iou_thres)
                    # Prepare PoseArray to hold the detected humans' 3D positions
                    person_poses = PoseArray()
                    person_poses.header.stamp = self.get_clock().now().to_msg()
                    person_poses.header.frame_id = self.base_frame
                    detection_count = self.results[0].boxes.shape[0]
                    for i in range(detection_count):
                        cls = int(self.results[0].boxes.cls[i].item())
                        print(cls)
                        if cls == 0 or cls == 1 or cls == 2:
                        # if cls == self.my_cls:
                            list_object = self.results[0].boxes.xyxy[i].cpu().numpy()
                            u = int((list_object[0]+list_object[2])/2)
                            v = int((list_object[1]+list_object[3])/2)
                            
                            # Get the depth value from the depth image at the center of the bounding box
                            depth = self.get_distance(self.depth, u, v)

                            if 0.0 < depth < self.permit_distance:
                                # Project the pixel (u, v) into a 3D ray
                                ray = self.camera_model.projectPixelTo3dRay((u, v))

                                # Multiply the ray by the depth to get the actual 3D position
                                X = ray[2] * depth
                                Y = ray[0] * depth
                                Z = ray[1] * depth
                                

                                # Create a Pose for the detected person
                                pose = Pose()
                                pose.position.x = X
                                pose.position.y = Y
                                pose.position.z = Z

                                # Add the Pose to the PoseArray
                                person_poses.poses.append(pose) # type: ignore

                    # Publish the PoseArray with all detected human positions
                    self.person_pub.publish(person_poses)
                    self.lock.release()
                    self.run_signal = False
                sleep(0.0001)
            except:
                self.exit_signal = True
        
    def listener_callback_image(self, data):
        self.lock.acquire()
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.lock.release()
        self.run_signal = True
        while self.run_signal:
            sleep(0.0001)
        self.annotated_frame = self.results[0].plot()
        cv2.imshow(self.cam_name, self.annotated_frame)
        key=cv2.waitKey(1)
        if key == 27:
            cv2.destroyWindow(self.cam_name)
            self.exit_signal = True
            
    
    def get_distance(self, data, u, v):
        Idx = u + data.width * v
        depths = struct.unpack('f' * (len(data.data) // 4), data.data)
        return depths[Idx]

    def listener_callback_image_depth(self, data):
        self.depth = data
        self.read_depth = True

    def listener_callback_image_info(self, data):
        if self.need_info:
            self.get_logger().info('Getting camera information')
            self.camera_model.fromCameraInfo(data)
            self.capture_thread = Thread(target=self.torch_thread, kwargs={'weights_yolo': self.weights_yolo, 'weights_custom': self.weights_custom,})
            self.capture_thread.start()
            self.need_info = False


    def publish_maker(self, position):
        markerArray = MarkerArray()
        num_object = len(position)
        for i in range(num_object):
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
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
            marker.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
            markerArray.markers.append(marker) # type: ignore
        if num_object < self.count_maker:  # delete older marker
            for i in range(num_object, self.count_maker):
                marker = Marker()
                marker.header.frame_id = self.base_frame
                marker.type = Marker.CYLINDER
                marker.action = Marker.DELETE
                marker.id = i
                markerArray.markers.append(marker) # type: ignore
        self.count_maker = num_object
        self.pub_maker.publish(markerArray)
        

    # def timer_callback(self):
    #     # print(self.need_info , self.read_depth, self.read_object)
    #     if self.need_info == False and self.read_depth == True:
    #         # print(merged_list)
    #         self.publish_maker(position)
    #         self.read_depth = False
    #         self.read_object = False

def main():
    rclpy.init()
    human_detection = HumanDetection()

    # Create an instance of MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    # Add the node to the executor
    executor.add_node(human_detection)

    try:
        rclpy.spin(human_detection)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Stopping Detection Process...")
    finally:
        # Clean up and shutdown
        executor.shutdown()
        human_detection.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()