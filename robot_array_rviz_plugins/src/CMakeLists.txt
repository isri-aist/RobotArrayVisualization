# This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

# This plugin includes Qt widgets, so we must include Qt. We'll use the version
# that rviz used so they are compatible.
message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
# make target_link_libraries(${QT_LIBRARIES}) pull in all required
# dependencies
set(QT_LIBRARIES Qt5::Widgets)

# I prefer the Qt signals and slots to avoid defining "emit", "slots", etc
# because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME} SHARED 
  ColorPropertySet.cpp
  RobotStateArrayVisual.cpp
  SingleRobotStateArrayDisplay.cpp
  MultiRobotStateArrayDisplay.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rviz2
  pluginlib
  rviz_common
  rviz_default_plugins
  robot_array_msgs
  class_loader
)

target_include_directories(${PROJECT_NAME} PRIVATE "${OGRE_PREFIX_DIR}/include")
target_link_libraries(${PROJECT_NAME} RBDyn::RBDyn RBDyn::Parsers
                      ${QT_LIBRARIES})