#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy

class LaserToOccupancyGrid(Node):
    def __init__(self):
        super().__init__('laser_to_occupancy_grid')
        self.qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,depth=1)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            self.qos)
        self.subscription  # prevent unused variable warning

        self.declare_parameter('map_resolution',0.25)
        self.map_resolution = self.get_parameter('map_resolution').get_parameter_value().double_value

        self.declare_parameter('map_origin_x',-5.0)
        self.map_origin_x = self.get_parameter('map_origin_x').get_parameter_value().double_value

        self.declare_parameter('map_origin_y',-5.0)
        self.map_origin_y = self.get_parameter('map_origin_y').get_parameter_value().double_value

        self.declare_parameter('cost_scaling_factor',1.0)
        self.cost_scaling_factor = self.get_parameter('cost_scaling_factor').get_parameter_value().double_value

        self.declare_parameter('gradient',5)
        self.gradient = self.get_parameter('gradient').get_parameter_value().integer_value

        self.map_size_x = int(2.0* abs(self.map_origin_x) / self.map_resolution)
        self.map_size_y = int(2.0* abs(self.map_origin_y) / self.map_resolution)
        
        self.map_data = np.zeros((self.map_size_x, self.map_size_y), dtype=np.int8)
        self.timer = self.create_timer(0.01, self.timer_callback)  # Timer for periodic execution
        self.map_publisher = self.create_publisher(OccupancyGrid, 'occupancy_grid_topic', 1)

    def fill_neighbors(self, array):
        obs = 100
        obs_prev = 100
        for k in range(self.gradient):
            obs = int(100 * math.exp(-(k+1) * self.cost_scaling_factor))
            for i in range(array.shape[0]):
                for j in range(array.shape[1]):
                    if array[i, j] == obs_prev:
                        if (i > 0)                and (array[i - 1, j] < array[i, j]):array[i - 1, j] = obs
                        if (i < array.shape[0]-1) and (array[i + 1, j] < array[i, j]):array[i + 1, j] = obs
                        if (j > 0)                and (array[i, j - 1] < array[i, j]):array[i, j - 1] = obs
                        if (j < array.shape[1]-1) and (array[i, j + 1] < array[i, j]):array[i, j + 1] = obs
            obs_prev = obs
        return array

    def laser_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        
        self.map_data = np.zeros((self.map_size_x, self.map_size_y), dtype=np.int8)

        for i, range_value in enumerate(ranges):
            obstacle_angle = i * angle_increment + angle_min
            if range_value != np.inf and (range_value < msg.range_max):
                x = int((range_value * math.sin(obstacle_angle) - self.map_origin_x)/ self.map_resolution)
                y = int((range_value * math.cos(obstacle_angle) - self.map_origin_y)/ self.map_resolution)
                if 0 <= x < self.map_size_x and 0 <= y < self.map_size_y:
                    self.map_data[x, y] = 100
        
        for i in range(int((-1.0- self.map_origin_x) /  self.map_resolution), int((1.00 - self.map_origin_x) /  self.map_resolution)):
            for j in range(int((-1.0 - self.map_origin_y) /  self.map_resolution), int((0.25 - self.map_origin_y) /  self.map_resolution)):
                self.map_data[i,j] =  0

    def timer_callback(self):
        self.map_data = self.fill_neighbors(self.map_data)
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header.frame_id = "base_scan"
        occupancy_grid_msg.info.resolution = self.map_resolution
        occupancy_grid_msg.info.width = self.map_size_x
        occupancy_grid_msg.info.height = self.map_size_y
        occupancy_grid_msg.info.origin.position.x = self.map_origin_x
        occupancy_grid_msg.info.origin.position.y = self.map_origin_y
        occupancy_grid_msg.data = self.map_data.flatten().tolist()
        self.map_publisher.publish(occupancy_grid_msg)

def main(args=None):
    rclpy.init(args=args)

    laser_to_occupancy_grid = LaserToOccupancyGrid()

    rclpy.spin(laser_to_occupancy_grid)

    laser_to_occupancy_grid.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
