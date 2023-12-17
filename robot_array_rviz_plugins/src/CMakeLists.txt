# This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

# This plugin includes Qt widgets, so we must include Qt. We'll use the version
# that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  # pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  # make target_link_libraries(${QT_LIBRARIES}) pull in all required
  # dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

# I prefer the Qt signals and slots to avoid defining "emit", "slots", etc
# because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

add_library(robot_array_rviz_plugins SingleRobotStateArrayDisplay.cpp
                                     SingleRobotStateArrayVisual.cpp)

target_include_directories(robot_array_rviz_plugins
                           PUBLIC ${catkin_INCLUDE_DIRS})

target_link_libraries(robot_array_rviz_plugins RBDyn::RBDyn RBDyn::Parsers
                      ${QT_LIBRARIES} ${catkin_LIBRARIES})
