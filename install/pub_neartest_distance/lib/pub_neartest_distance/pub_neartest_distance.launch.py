import os, launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pub_neartest_distance'),
        'config',
        'params.yaml'
        )

    node = launch_ros.actions.Node(
        package='pub_neartest_distance',
        executable='pub_neartest_distance.py',
        output='screen',
        emulate_tty=True,
        parameters=[config])
    
    return LaunchDescription([
        node
    ])
