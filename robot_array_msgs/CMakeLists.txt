cmake_minimum_required(VERSION 3.1)
project(robot_array_msgs)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs
                                        geometry_msgs)

add_message_files(FILES RobotState.msg SingleRobotStateArray.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
