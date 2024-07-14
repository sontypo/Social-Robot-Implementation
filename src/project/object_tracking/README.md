# Deep Learning for Object Detection and Object Following
This package contains essential commands to understand the basic principles of applying deep learning to object detection and tracking using YOLOv8.

<img src=media/PID.gif width="1040">

## Simulation
If you want to try in the simulator, make sure Gazebo is enabled by running the following command:
```
ros2 launch run_sensor object_tracking_gazebo.launch.py
ros2 launch visual_control_v1 irobot_PID.launch.py
```
- scout_simulation
- object_detection
- pointcloud_to_laserscan
- obstacle_detect
- point_cal


## Real
If you want to try it out in the real world, make sure the program is enabled by executing the following command:
```
ros2 launch run_sensor object_tracking_real.launch.py
ros2 launch visual_control_v1 irobot_PID.launch.py
```
- scout_base
- zed_wrapper
- velodyne
- point_cal
- object_detection
- obstacle_detect

## 1. Sensor
Make sure the required sensors are turned on.
```
ros2 launch zed_wrapper zedx.launch.py
ros2 launch velodyne velodyne-all-nodes-VLP16-composed-launch.py
```

## 2. Object Detection
The robot uses the YOLOv8 model for object recognition. If you wish to change the model, navigate to the ``object_detection/scripts/weights`` directory and rename the model file accordingly. To train for the custom model, please refer by [this tutorial](../train_model_yolov8).

```
ros2 launch object_detection object_detection.launch.py #for real robot
ros2 launch object_detection object_detection_gazebo.launch.py #for gazebo
```

## 3. Goal Calculation
By inputting object information and laserscan data (from velodyne), the target point that the robot will reach is calculated. The calculation process takes into account the distance to obstacles and the angle between the robot and the object.
```
ros2 launch point_cal point_cal_real.launch.py #for real robot
ros2 launch point_cal point_cal_gazebo.launch.py #for gazebo
```

## 4. Object Tracking
After successfully detecting objects, a navigation framework is employed based on the relative positions of the objects for tracking. The robot will halt if it cannot detect any objects along its path.
```
ros2 launch visual_control_v1 irobot_PID.launch.py 
```