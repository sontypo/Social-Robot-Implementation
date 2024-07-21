import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

ROBOT = os.environ.get('ROBOT', "diff_bot")

def generate_launch_description():
    pkg_share = get_package_share_directory('differential_robot')
    # world_path = os.path.join(get_package_share_directory('ros_ign_gazebo'), 'worlds', 'empty.sdf')
    # default_world_path = os.path.join(pkg_share, 'worlds', 'simple_actor.world')
    
    robot_desc = os.path.join(pkg_share, 'urdf', 'diffbot_classic_gazebo.urdf')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    world = os.path.join(
        pkg_share,
        'worlds',
        'single_actor.world'
    )

    # gzserver_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
    #     ),
    #     launch_arguments={'world': world}.items()
    # )

    # gzclient_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
    #     )
    # )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': robot_desc}])
    
    # Publish the joint states of the robot
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher')
    
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', ROBOT,
            '-file', robot_desc,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.1'
        ],
        output='screen',
    )
    
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world
            }.items()
        )

    ld = LaunchDescription()

    # Add the commands to the launch description
    # ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(start_joint_state_publisher_cmd)
    # ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(gazebo)
    
    return ld