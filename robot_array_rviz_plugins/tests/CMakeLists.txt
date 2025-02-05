find_package(launch_testing_ament_cmake)

add_launch_test(scripts/testSingleRobotStateArrayClient.py)

add_launch_test(scripts/testMultiRobotStateArrayClient.py)

install(PROGRAMS scripts/testRobotDescriptionMap.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY rviz scripts DESTINATION share/${PROJECT_NAME}/tests)
