cmake_minimum_required(VERSION 3.22)
project(robot_array_rviz_plugins)

set(CMAKE_CXX_STANDARD 17)

find_package(RBDyn REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(robot_array_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_subdirectory(src)

install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

# plugin export #
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package()