import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    config_rviz2 = os.path.join(
        get_package_share_directory('object_detection'),
        'rviz2',
        'object_detection_rviz2.rviz'
        )
        
     # Rviz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='object_detection_rviz2',
        output='screen',
        arguments=[["-d"], [config_rviz2]],
    )

    config = os.path.join(
        get_package_share_directory('object_detection'),
        'config',
        'params_gazebo.yaml'
        )
    
    node_name='object_detection'

    node = launch_ros.actions.Node(
        package='object_detection',
        executable='object_detection.py',
        name=node_name,
        output='screen',
        emulate_tty=True,
        parameters=[config])
    
    real_position='real_objection'

    real_node = launch_ros.actions.Node(
        package='object_detection',
        executable='real_objection.py',
        name=real_position,
        output='screen',
        emulate_tty=True,
        parameters=[config])
    

    return LaunchDescription([
        node,real_node,rviz2_node
    ])
