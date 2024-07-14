#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import  time, os
from sensor_msgs.msg import Image
import torch
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
from threading import Lock, Thread
from time import sleep
from std_msgs.msg import Int16MultiArray
# torch.cuda.set_device(0) # Set to your desired GPU number
class object_detection(Node):

    def __init__(self) :
        """
        Initialize the node, open serial port
        """        
        # Init node
        super().__init__('object_detection')
        # parameters
        self.declare_parameter('cam_name','cam_name')
        self.cam_name = self.get_parameter('cam_name').get_parameter_value().string_value

        self.declare_parameter('topic_camera','topic_camera')
        self.topic_camera = self.get_parameter('topic_camera').get_parameter_value().string_value

        self.declare_parameter('model_yolo','model_yolo')
        self.model_yolo = self.get_parameter('model_yolo').get_parameter_value().string_value

        self.declare_parameter('model_yolo_custom','model_yolo_custom')
        self.model_yolo_custom = self.get_parameter('model_yolo_custom').get_parameter_value().string_value

        self.declare_parameter('topic_object','topic_object')
        self.topic_object = self.get_parameter('topic_object').get_parameter_value().string_value

        self.my_cls = self.declare_parameter('my_cls',0).get_parameter_value().integer_value

        self.data_object=Int16MultiArray()

        self.bridge = CvBridge()
        self.dirPath = os.path.dirname(os.path.realpath(__file__))
        self.weights_yolo = "/home/saun/ctrl_mr_dynamic/src/project/object_tracking/object_detection/scripts/weights/"+self.model_yolo
        self.weights_custom = "/home/saun/ctrl_mr_dynamic/src/project/object_tracking/object_detection/scripts/weights/"+self.model_yolo_custom
        self.lock = Lock()
        self.exit_signal=False
        self.run_signal=False

        self.subscription_camera = self.create_subscription(
            Image,
            self.topic_camera,
            self.listener_callback_image,
            1)
        
        self.pub_object = self.create_publisher(Int16MultiArray,  self.topic_object, 2)
        self.capture_thread = Thread(target=self.torch_thread, kwargs={'weights_yolo': self.weights_yolo, 'weights_custom': self.weights_custom,})
        self.capture_thread.start()
        
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
                    detection_count = self.results[0].boxes.shape[0]
                    for i in range(detection_count):
                        cls = int(self.results[0].boxes.cls[i].item())
                        print(cls)
                        if cls == 0 or cls == 1 or cls == 2:
                        # if cls == self.my_cls:
                            list_object = self.results[0].boxes.xyxy[i].cpu().numpy()
                            self.x_center = int((list_object[0]+list_object[2])/2)
                            self.y_center = int((list_object[1]+list_object[3])/2)
                            object_position = np.array([self.x_center, self.y_center]).tolist()
                            break
                            
                    self.data_object.data = object_position
                    self.pub_object.publish(self.data_object)

                    # list_object = self.results[0].boxes.xyxy
                    # self.x_center=((list_object[:,0]+list_object[:,2])/2).int()
                    # self.y_center=((list_object[:,1]+list_object[:,3])/2).int()
                    # object_position = torch.cat((self.x_center,self.y_center)).tolist()
                    # self.data_object.data = object_position
                    # self.pub_object.publish(self.data_object)
                    # print(object_position)
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

def start():
    rclpy.init()
    object = object_detection()
    try:
        rclpy.spin(object)
        object.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Closing object detection...")
    
   

if __name__ == '__main__':
    start()