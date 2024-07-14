# Fundamentals of Mobile Robot Navigation, Path Planning, and Obstacle Avoidance
![](media/SLAM.gif)

This package includes essential commands for understanding the fundamentals of mobile robot navigation, path planning, and obstacle avoidance.

If you want to try in the simulator, make sure Gazebo is enabled by running the following command:
```
ros2 launch scout_simulation simulation.launch.py
```


## 1. Localization
In this project, the [SLAM toolbox](https://github.com/SteveMacenski/slam_toolbox) is utilized as a localization tool for the robot through the SLAM algorithm, encompassing mapping and localization steps.
### 1.1 Mapping
If you are using the real robot, please run the command:
```
ros2 launch scout_base scout_mini_omni_base.launch.py
ros2 launch velodyne velodyne-all-nodes-VLP16-composed-launch.py
```
Mapping involves creating a representation of the environment using sensor data. You can use a SLAM algorithm for this purpose. Here's a generic guide for mapping:
```
ros2 launch autonoumous_navigation slam_real.launch.py
ros2 launch autonoumous_navigation slam_gazebo.launch.py
```
Use the ``teleop_twist_keyboard package`` for teleoperation in a simulated environment.
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
In the real world, please use Remote control to move the robot.
### 1.2 Save the map
Execute the following command to save the map, Please note that this command will generate two files, namely map.pgm and map.yaml, in the specified directory (in this case, ~/map). Afterward, you can copy these two files to the ``maps`` directory of the ``autonomous_navigation`` package.
```
ros2 run nav2_map_server map_saver_cli -f ~/map
```
Or click directly in slam_toolbox to save the map.


## 2. Navigation
Before starting navigation, ensure the correct map name is specified in the launch file, obtain the map, and then use the provided command; remember to select the initial starting point for the robot.

```
ros2 launch autonoumous_navigation navigation_real.launch.py 
ros2 launch autonoumous_navigation navigation_gazebo.launch.py
```
After the Rviz frame appears, you can observe the local and global planning, as well as the obstacles that appear in the environment.
## 3. Assignment
Develop a ROS2 control program to generate waypoints for the robot to pass through by creating a program that publishes to the ``goal_pose`` topic with the message type ``PoseStamped``. Additionally, utilize the robot's current position to validate the waypoints.
