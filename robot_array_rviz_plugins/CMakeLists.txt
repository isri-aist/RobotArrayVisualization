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
find_package(launch_testing_ament_cmake)

ament_export_dependencies(class_loader REQUIRED)
ament_export_dependencies(pluginlib REQUIRED)
ament_export_dependencies(rclcpp REQUIRED)
ament_export_dependencies(std_msgs REQUIRED)
ament_export_dependencies(rviz2 REQUIRED)
ament_export_dependencies(Eigen3 REQUIRED)

###############
# Qt settings #
###############
find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
macro(qt_wrap_ui)
  qt5_wrap_ui(${ARGN})
endmacro()
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
add_definitions(-DQT_NO_KEYWORDS)
# cmake settings #
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
# I prefer the Qt signals and slots to avoid defining "emit", "slots", etc
# because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

#########
# Build #
#########
set(SOURCE_FILES
  src/ColorPropertySet.cpp
  src/RobotStateArrayVisual.cpp
  src/SingleRobotStateArrayDisplay.cpp
  src/MultiRobotStateArrayDisplay.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
ament_target_dependencies(${PROJECT_NAME}
  std_msgs
  rviz2
  Qt5
  pluginlib
  rviz_common
  rviz_default_plugins
  robot_array_msgs
  class_loader
)

target_include_directories(${PROJECT_NAME} PRIVATE "${OGRE_PREFIX_DIR}/include")
target_link_libraries(${PROJECT_NAME} RBDyn::RBDyn RBDyn::Parsers)

###########
# Install #
###########
install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

###########
#  Test   #
###########
if(BUILD_TESTING)
  add_launch_test(
    tests/scripts/testSingleRobotStateArrayClient.py
  )
  
  install(PROGRAMS tests/scripts/testRobotDescriptionMap.py
    DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY tests DESTINATION 
    DESTINATION share/${PROJECT_NAME})
    
endif()

# plugin export #
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package()