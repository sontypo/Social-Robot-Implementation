import rclpy
import os
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray 
import pandas as pd
import numpy as np

class NodeLogger(Node):
    def __init__(self):
        super().__init__('odom_logger')
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.human_subscription = self.create_subscription(Float32MultiArray, '/person_array', self.human_position_callback, 10)
        self.force_subscription = self.create_subscription(Float32MultiArray, '/applied_force', self.force_callback, 10)
        
        # Dataframes for storing the position and velocity
        self.position_data = pd.DataFrame(columns=['x', 'y'])
        self.velocity_data = pd.DataFrame(columns=['linear.x', 'linear.y', 'angular.z'])
        
        # Dataframe for storing the human position
        self.human_position_data = pd.DataFrame(columns=['x', 'y', 'z'])
        
        # Dataframe for storing the applied force values
        self.force_values = pd.DataFrame(columns=['Fm', 'Fn'])

    def odom_callback(self, msg: Odometry):
        # Extract position data
        position = msg.pose.pose.position
        new_position = pd.DataFrame({
            'x': [position.x],
            'y': [position.y]
        })
        
        # Check if the position_data DataFrame is empty and concatenate only if it’s not
        if not new_position.isna().all().all():
            self.position_data = pd.concat([self.position_data, new_position], ignore_index=True)
        
        # Extract velocity data
        velocity = msg.twist.twist
        new_velocity = pd.DataFrame({
            'linear.x': [velocity.linear.x],  \
            'linear.y': [velocity.linear.y],  \
            'angular.z': [velocity.angular.z] \
        })
        
        # Check if the velocity_data DataFrame is empty and concatenate only if it’s not
        if not new_velocity.isna().all().all():
            self.velocity_data = pd.concat([self.velocity_data, new_velocity], ignore_index=True)
            
    def human_position_callback(self, msg: Float32MultiArray):
        # Extract position data
        if len(msg.data) != 0:
            human_position = np.array(msg.data)
            new_human_position = pd.DataFrame({
                'x': [human_position[0]],
                'y': [human_position[1]],
                'z': [human_position[2]]
            })
            
            # Check if the position_data DataFrame is empty and concatenate only if it’s not
            if not new_human_position.isna().all().all():
                self.human_position_data = pd.concat([self.human_position_data, new_human_position], ignore_index=True)
            
    def force_callback(self, msg: Float32MultiArray):
        # Extract force values
        force_value = np.array(msg.data)
        new_force_value = pd.DataFrame({
            'Fm': [force_value[0]],
            'Fn': [force_value[1]]
        })
        
        # Check if the position_data DataFrame is empty and concatenate only if it’s not
        if not new_force_value.isna().all().all():
            self.force_values = pd.concat([self.force_values, new_force_value], ignore_index=True)

    def save_data(self, input_directory):        
        # Check if the directory exists, if not, create it
        if not os.path.exists(input_directory):
            os.makedirs(input_directory)  # Creates the new directory if it doesn't exist

        # Create full file paths
        position_file_path = os.path.join(input_directory, 'position_data.csv')
        velocity_file_path = os.path.join(input_directory, 'velocity_data.csv')
        human_position_file_path = os.path.join(input_directory, 'human_position_data.csv')
        force_values_file_path = os.path.join(input_directory, 'force_values.csv')

        # Save the data to CSV files in the specified directory
        self.position_data.to_csv(position_file_path, index=False)
        self.velocity_data.to_csv(velocity_file_path, index=False)
        self.human_position_data.to_csv(human_position_file_path, index=False)
        self.force_values.to_csv(force_values_file_path, index=False)

def main(args=None):
    rclpy.init(args=args)
    node = NodeLogger()
    
    # Get directory input from the user
    save2folder = input("Create the folder to save CSV files (will be created if it doesn't exist): ")
    
    # Create an instance of MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    # Add the node to the executor
    executor.add_node(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Save data when the node is stopped
        node.save_data(save2folder)
    finally:
        # Clean up and shutdown
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
