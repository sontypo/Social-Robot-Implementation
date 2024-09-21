import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('force_control_exp')
    
    config = os.path.join(pkg_share, 'params', 'params.yaml')
    
    force_control_exp_node = Node(
        package='force_control_exp',
        executable='force_control_exp_node',
        name='force_control_exp_node',
        output='screen',
        parameters=[config])

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(force_control_exp_node)
    
    return ld