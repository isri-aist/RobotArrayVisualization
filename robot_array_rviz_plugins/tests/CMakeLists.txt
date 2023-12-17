find_package(rostest REQUIRED)

set(RobotArrayRvizPlugins_rostest_list TestSingleRobotStateArrayDisplay)

foreach(NAME IN LISTS RobotArrayRvizPlugins_rostest_list)
  add_rostest(test/${NAME}.test)
endforeach()
