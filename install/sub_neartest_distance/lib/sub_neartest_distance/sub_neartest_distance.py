#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Pose2D


class ref_calculate(Node):

    def __init__(self) :
        """
        Initialize the node, open serial port
        """        
        super().__init__('sub_neartest_distance')

        self.qos = QoSProfile(depth=1)

        # parameters
        self.laser_max_dis = self.declare_parameter('laser_max_dis', 5.0).get_parameter_value().double_value

        self.point = Pose2D()

        # Subscribe point topic type: Pose2D, topic name:'/near_point'
        self.sub = self.create_subscription(Pose2D, '/near_point', self.call_back, self.qos) ###

    def call_back(self, msg:Pose2D):
        # get self.point from msg 
        self.point = msg ###
        if self.point.x >= 999:
            print('Out of ', round(self.laser_max_dis,1), ' meters!')
        else:
            dis = math.hypot(self.point.x,self.point.y)
            print('point: (', round(self.point.x,2), ',', round(self.point.y,2), ')\tdistance: ', round(dis,2))

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