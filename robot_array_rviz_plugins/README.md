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
Display an array of single robots from messages of [`robot_array_msgs/RobotStateArray`](../robot_array_msgs/msg/RobotStateArray.msg).

#### Sample
```bash
$ rostest robot_array_rviz_plugins TestSingleRobotStateArrayDisplay.test --text
```

https://github.com/isri-aist/RobotArrayVisualization/assets/6636600/4eae6e07-738d-490e-b254-1b02bcb04e8c
