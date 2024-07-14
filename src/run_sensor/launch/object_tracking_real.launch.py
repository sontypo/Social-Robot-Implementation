import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    scout_dir = get_package_share_directory('scout_base')
    scout_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(scout_dir + '/launch/scout_mini_omni_base.launch.py'))

    laser_dir = get_package_share_directory('velodyne')
    laser_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(laser_dir + '/launch/velodyne-all-nodes-VLP16-composed-launch.py'))

    zed_dir = get_package_share_directory('zed_wrapper')
    zed_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(zed_dir + '/launch/zedx.launch.py'))

    object_dir = get_package_share_directory('object_detection')
    object_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(object_dir + '/launch/object_detection.launch.py'))
    
    cost_map_dir = get_package_share_directory('obstacle_detect')
    cost_map_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(cost_map_dir + '/launch/cost_map.launch.py'))

    point_cal_dir = get_package_share_directory('point_cal')
    point_cal_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(point_cal_dir + '/launch/point_cal_real.launch.py'))
     
    return launch.LaunchDescription([
        scout_launch,
        laser_launch,
        zed_launch,
        object_launch,
        cost_map_launch,
        point_cal_launch
    ])
