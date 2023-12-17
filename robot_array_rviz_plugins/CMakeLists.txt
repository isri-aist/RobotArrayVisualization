cmake_minimum_required(VERSION 3.1)
project(robot_array_rviz_plugins)

set(CMAKE_CXX_STANDARD 17)

option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

find_package(catkin REQUIRED COMPONENTS roscpp rviz robot_array_msgs)

find_package(RBDyn REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp rviz robot_array_msgs LIBRARIES
               robot_array_rviz_plugins)

add_subdirectory(src)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

# if(INSTALL_DOCUMENTATION) add_subdirectory(doc) endif()
