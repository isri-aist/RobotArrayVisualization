cmake_minimum_required(VERSION 3.22)
project(robot_array_rviz_plugins)

set(CMAKE_CXX_STANDARD 17)

# Enable CMake policy CMP0148 for github actions
if(POLICY CMP0148) 
cmake_policy(SET CMP0148 OLD)
endif()

###################
# Depend packages #
###################
find_package(RBDyn REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(robot_array_msgs REQUIRED)

ament_export_dependencies(class_loader REQUIRED)
ament_export_dependencies(pluginlib REQUIRED)
ament_export_dependencies(rclcpp REQUIRED)
ament_export_dependencies(std_msgs REQUIRED)
ament_export_dependencies(rviz2 REQUIRED)
ament_export_dependencies(Eigen3 REQUIRED)

add_subdirectory(src)

###########
# Install #
###########
install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# plugin export #
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package()