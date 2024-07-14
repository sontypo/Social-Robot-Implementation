#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs  # Import to use the do_transform_pose function
import time
import math 

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer_ = self.create_timer(0.01, self.publish_goal_pose)
        self.goal_poses = self.generate_goal_poses()
        self.current_goal_index = 0

    def generate_goal_poses(self):
        # Create a list containing 4 pre-defined goal poses
        goal_poses = []
        goal_poses.append(self.create_pose_stamped(2.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        goal_poses.append(self.create_pose_stamped(0.0, 2.0, 0.0, 0.0, 0.0, 1.57))
        goal_poses.append(self.create_pose_stamped(-2.0, 0.0, 0.0, 0.0, 0.0, 3.14))
        goal_poses.append(self.create_pose_stamped(0.0, -2.0, 0.0, 0.0, 0.0, -1.57))
        return goal_poses

    def create_pose_stamped(self, x, y, z, qx, qy, qz):
        # Helper function to create a PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = 1.0  # Quaternions must be normalized
        pose_stamped.header.frame_id = 'map'
        return pose_stamped

    def publish_goal_pose(self):
        try:
            # Get the transform information between "map" and "base_link"
            transform_stamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            print(transform_stamped)
            # Get the current goal and transform it from "map" to "base_link"
            current_goal = self.goal_poses[self.current_goal_index]
            # Publish the transformed goal
            self.publisher_.publish(current_goal)
            #Check if the robot has reached the current goal
            deltax = transform_stamped.transform.translation.x - current_goal.pose.position.x
            deltay = transform_stamped.transform.translation.y - current_goal.pose.position.y
            distance_to_goal = math.sqrt(deltax**2 + deltay**2)
            print(distance_to_goal)
            if distance_to_goal < 0.3:  # Adjust the threshold value as needed
                self.get_logger().info(f'Reached goal {self.current_goal_index + 1}, moving to the next goal...')
                self.current_goal_index = (self.current_goal_index + 1) % len(self.goal_poses)

        except Exception as e:
           pass

def main(args=None):
    rclpy.init(args=args)

    goal_pose_publisher = GoalPosePublisher()

    try:
        rclpy.spin(goal_pose_publisher)
    except KeyboardInterrupt:
        pass

    goal_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
