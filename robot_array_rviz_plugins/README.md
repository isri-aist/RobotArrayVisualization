# [robot_array_rviz_plugins](https://github.com/isri-aist/RobotArrayVisualization/tree/main/robot_array_rviz_plugins)
Rviz plugins for visualization of robot arrays

## Install

### Requirements
- Compiler supporting C++17
- Tested on `Ubuntu 20.04 / ROS Noetic`

### Dependencies
This package depends on
- [RBDyn](https://github.com/jrl-umi3218/RBDyn)

### Installation procedure
Clone this repository into the catkin workspace and build it as a catkin package (i.e., `catkin build` or `catkin_make` command).

## Plugins
### SingleRobotStateArray
Display an array of single robots (i.e., robots with the same URDF model) from messages of [`robot_array_msgs/RobotStateArray`](../robot_array_msgs/msg/RobotStateArray.msg).

#### Sample
```bash
$ launch_test `ros2 pkg prefix robot_array_rviz_plugins`/share/robot_array_rviz_plugins/tests/scripts/testSingleRobotStateArrayClient.py
```

https://github.com/isri-aist/RobotArrayVisualization/assets/6636600/4eae6e07-738d-490e-b254-1b02bcb04e8c

### MultiRobotStateArray
Display an array of multiple types of robots (i.e., robots with different URDF models) from messages of [`robot_array_msgs/RobotStateArray`](../robot_array_msgs/msg/RobotStateArray.msg).

#### Sample
```bash
$ launch_test `ros2 pkg prefix robot_array_rviz_plugins`/share/robot_array_rviz_plugins/tests/scripts/testMultiRobotStateArrayClient.py
```

https://github.com/isri-aist/RobotArrayVisualization/assets/6636600/5e33c0a1-9f30-4e1c-8373-fadb73d45124
