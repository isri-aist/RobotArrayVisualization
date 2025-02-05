#! /usr/bin/env python3

import sys
import unittest
import copy
import numpy as np
import rclpy
from robot_array_msgs.msg import RobotState, RobotStateArray

import os
import xacro
import pytest
import launch
import launch_testing
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def create_urdf(xacro_file, mappings):
    urdf = xacro.process_file(xacro_file, mappings=mappings)

    urdf_file = urdf.toprettyxml(indent="  ")

    return urdf_file


@pytest.mark.launch_test
def generate_test_description():
    robot_array_rviz_plugins_package = FindPackageShare(
        package="robot_array_rviz_plugins"
    ).find("robot_array_rviz_plugins")

    rviz_config_file = os.path.join(
        robot_array_rviz_plugins_package,
        "tests",
        "rviz",
        "TestSingleRobotStateArrayDisplay.rviz",
    )

    fr3_mappings = {
        "load_gripper": "false",
        "arm_id": "fr3",
    }

    fr3_file_path = os.path.join(
        FindPackageShare(package="franka_description").find("franka_description"),
        "robots",
        "fr3",
        "fr3.urdf.xacro",
    )

    fr3_urdf = create_urdf(fr3_file_path, fr3_mappings)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": fr3_urdf},
        ],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )

    context = {}

    return (
        launch.LaunchDescription(
            [robot_state_publisher, rviz2_node, launch_testing.actions.ReadyToTest()]
        ),
        context,
    )


class TestSingleRobotStateArrayClient(unittest.TestCase):
    def __init__(self, *args):
        super().__init__(*args)

    def test(self):
        rclpy.init(args=sys.argv)
        node = rclpy.create_node("client")

        pub = node.create_publisher(RobotStateArray, "robot_state_arr", 1)

        default_robot_state_msg = RobotState()
        default_robot_state_msg.name = "fr3"
        default_robot_state_msg.root_pose.header.frame_id = "map"
        default_robot_state_msg.root_pose.pose.position.x = 0.0
        default_robot_state_msg.root_pose.pose.position.y = 0.0
        default_robot_state_msg.root_pose.pose.position.z = 0.0
        default_robot_state_msg.root_pose.pose.orientation.x = 0.0
        default_robot_state_msg.root_pose.pose.orientation.y = 0.0
        default_robot_state_msg.root_pose.pose.orientation.z = 0.0
        default_robot_state_msg.root_pose.pose.orientation.w = 1.0
        for j in range(1, 8):
            default_robot_state_msg.joint_name_list.append("fr3_joint{}".format(j))
            default_robot_state_msg.joint_pos_list.append(
                -0.5 * np.pi if j == 4 else 0.0
            )

        rate = node.create_rate(1000)
        start_t = rclpy.clock.Clock().now().nanoseconds / 1e9
        fail_count = 0
        fail_count_thre = 1000
        while rclpy.ok():
            t = rclpy.clock.Clock().now().nanoseconds / 1e9

            robot_state_arr_msg = RobotStateArray()

            for i in range(5):
                robot_state_msg = copy.deepcopy(default_robot_state_msg)
                robot_state_msg.root_pose.pose.position.x = np.sin(0.5 * t)
                t_i = t - 0.4 * i
                robot_state_msg.joint_pos_list[0] = 2.0 * np.cos(0.5 * t_i)
                robot_state_msg.joint_pos_list[1] = 1.0 * np.sin(1.0 * t_i)
                robot_state_msg.joint_pos_list[2] = 1.0 * np.cos(1.0 * t_i)
                robot_state_arr_msg.robot_states.append(robot_state_msg)

            pub.publish(robot_state_arr_msg)

            if rosnode_ping(node, "rviz2", max_count=1):
                fail_count = 0
            else:
                fail_count += 1
            if fail_count >= fail_count_thre:
                node.get_logger().error("rviz is not launched")
                sys.exit(1)

            if t - start_t > 10.0:
                break

            rclpy.spin_once(node)
            rate.sleep()


def rosnode_ping(node, node_name, max_count):
    node_names = node.get_node_names()
    if node_name in node_names:
        return True
    else:
        return False
