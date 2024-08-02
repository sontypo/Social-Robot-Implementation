# Social-Robot-Implementation

Maintainer: <hongsonnguyen.haui@gmail.com>
---------------------------------------------------------------

[![Watch the video]](https://github.com/sontypo/Social-Robot-Implementation/blob/master/simulation_program_august2.mp4)

## Requirement
- OS: Ubuntu 20.04 or later version
- Environment: ROS2, Gazebo classic
- Python libraries: torch, ultralytics...
---------------------------------------------------------------

## Note: Before running the simulation program
- First, you need to rebuild the workspace on your computer
- There are some code in the workspace contain the previous user's paths, so you need to identify the correct paths on your computer and modify them accordingly.
---------------------------------------------------------------

## Steps to run the simulation program on Gazebo
- Step 1: Initializing differential-drive mobile robot and single actor in Gazebo, make sure that you always "source install/setup.bash" 
  > ros2 launch differential_robot classic_spawn_diffbot.launch.py
- Step 2: Open another terminal, launching the object detetion model for human recognition task
  > ros2 launch object_detection object_detection_gazebo.launch.py
- Step 3: Open one more tab on your terminal, then running the force_computation node for dynamic model
  > ros2 run force_control_exp force_control_exp_node
