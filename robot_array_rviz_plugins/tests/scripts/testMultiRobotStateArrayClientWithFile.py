#! /usr/bin/env python3

import sys
import unittest
import copy
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from robot_array_msgs.msg import RobotState, RobotStateArray

import os
import pytest
import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


@pytest.mark.launch_test
def generate_test_description():
    robot_array_rviz_plugins_package = FindPackageShare(
            package="robot_array_rviz_plugins").find(
            "robot_array_rviz_plugins")

    rviz_config_file = os.path.join(
        robot_array_rviz_plugins_package,
        "tests",
        "rviz",
        "TestMultiRobotStateArrayDisplayWithFile.rviz"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )

    context = {}

    return launch.LaunchDescription([
        rviz2_node,
        launch_testing.actions.ReadyToTest()
    ]), context


class testMultiRobotStateArrayClientwithFile(unittest.TestCase):
    def __init__(self, *args):
        super().__init__(*args)

    def test(self):
        rclpy.init(args=sys.argv)
        node = rclpy.create_node("client")

        pub = node.create_publisher(RobotStateArray, "robot_state_arr", 1)

        default_pose_st_msg = PoseStamped()
        default_pose_st_msg.header.frame_id = "map"
        default_pose_st_msg.pose.position.x = 0.0
        default_pose_st_msg.pose.position.y = 0.0
        default_pose_st_msg.pose.position.z = 0.0
        default_pose_st_msg.pose.orientation.x = 0.0
        default_pose_st_msg.pose.orientation.y = 0.0
        default_pose_st_msg.pose.orientation.z = 0.0
        default_pose_st_msg.pose.orientation.w = 1.0

        default_robot_state_msg_panda = RobotState()
        default_robot_state_msg_panda.name = "fr3"
        default_robot_state_msg_panda.root_pose = default_pose_st_msg
        panda_joint_name_list = ["fr3_joint{}".format(i) for i in range(1, 8)]
        panda_joint_pos_list = [0.0, 0.0, 0.0, -0.5 * np.pi, 0.0, 0.0, 0.0]
        for jname, jpos in zip(panda_joint_name_list, panda_joint_pos_list):
            default_robot_state_msg_panda.joint_name_list.append(jname)
            default_robot_state_msg_panda.joint_pos_list.append(jpos)

        default_robot_state_msg_ur5e = RobotState()
        default_robot_state_msg_ur5e.name = "ur5e"
        default_robot_state_msg_ur5e.root_pose = default_pose_st_msg
        ur5e_joint_name_list = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        ur5e_joint_pos_list = [0.0, -0.5, 1.0, 0.0, 0.0, 0.0]
        for jname, jpos in zip(ur5e_joint_name_list, ur5e_joint_pos_list):
            default_robot_state_msg_ur5e.joint_name_list.append(jname)
            default_robot_state_msg_ur5e.joint_pos_list.append(jpos)

        rate = node.create_rate(1000)
        start_t = rclpy.clock.Clock().now().nanoseconds / 1e9
        fail_count = 0
        fail_count_thre = 1000
        while rclpy.ok():
            t = rclpy.clock.Clock().now().nanoseconds / 1e9

            robot_state_arr_msg = RobotStateArray()

            for i in range(5):
                robot_state_msg = copy.deepcopy(default_robot_state_msg_panda)
                robot_state_msg.root_pose.pose.position.x = np.sin(0.5 * t)
                t_i = t - 0.4 * i
                robot_state_msg.joint_pos_list[0] = 2.0 * np.cos(0.5 * t_i)
                robot_state_msg.joint_pos_list[1] = 1.0 * np.sin(1.0 * t_i)
                robot_state_msg.joint_pos_list[2] = 1.0 * np.cos(1.0 * t_i)
                robot_state_arr_msg.robot_states.append(robot_state_msg)

            for i in range(3):
                robot_state_msg = copy.deepcopy(default_robot_state_msg_ur5e)
                robot_state_msg.root_pose.pose.position.y = -1.0
                robot_state_msg.joint_pos_list[0] = (
                    2.0 * np.pi * (2.0 * np.random.rand() - 1.0)
                )
                robot_state_msg.joint_pos_list[1] = np.pi * (
                    2.0 * np.random.rand() - 1.0
                )
                robot_state_msg.joint_pos_list[2] = np.pi * (
                    2.0 * np.random.rand() - 1.0
                )
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
