import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('point_cal'),
        'config',
        'params_gazebo.yaml'
        )

    node = launch_ros.actions.Node(
        name = 'point_cal_simulation',
        package='point_cal',
        executable='point_cal.py',
        output='screen',
        emulate_tty=True,
        parameters=[config])
    
    return LaunchDescription([
        node
    ])
