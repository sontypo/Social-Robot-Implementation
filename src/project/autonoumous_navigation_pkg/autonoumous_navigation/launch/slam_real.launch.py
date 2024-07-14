import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    rviz_config_dir = os.path.join(
        get_package_share_directory('autonoumous_navigation'),
        'rviz',
        'slam.rviz')
        
    pointcloud_laser=IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('pointcloud_to_laserscan'),
            '/launch/pointcloud_to_laserscan_launch.py'
        ])
    )
    
    slam=IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('autonoumous_navigation'),
            '/launch/online_async_launch_real.py'
        ])
    )
    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='slam',
        output='screen',
        arguments=[["-d"], [rviz_config_dir]],
    )

    return LaunchDescription([
    	slam,
    	rviz2_node
       
    ])
