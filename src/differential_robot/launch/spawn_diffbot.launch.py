import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT = os.environ.get('ROBOT', "diff_bot")
ROBOT_FORMAT_FILE = os.environ.get('ROBOT_FORMAT_FILE', 'urdf')

def generate_launch_description():
    pkg_share = get_package_share_directory('differential_robot')
    # world_path = os.path.join(get_package_share_directory('ros_ign_gazebo'), 'worlds', 'empty.sdf')
    
    if ROBOT_FORMAT_FILE == "urdf":
        robot_desc = os.path.join(pkg_share, 'urdf', 'diffbot_realsize.urdf')
    elif ROBOT_FORMAT_FILE == "sdf":
        sdf_file = os.path.join(pkg_share, 'models', 'diffbot_irl.sdf')
        
        with open(sdf_file, 'r') as infp:
            robot_desc = infp.read()
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                # launch_arguments={'ign_args': '-r ' + world_path}.items(),
                )
    
    spawn_entity = Node(package='ros_gz_sim',
                executable='create',
                arguments=['-file', robot_desc, '-name', 'diff_bot'],
                output='screen')
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[robot_desc])
            

    return LaunchDescription([gazebo, spawn_entity, robot_state_publisher])