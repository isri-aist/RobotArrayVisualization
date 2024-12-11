#! /usr/bin/env python

import os
import sys
import unittest
import copy
import numpy as np
import rclpy
from robot_array_msgs.msg import RobotState, RobotStateArray


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
            default_robot_state_msg.joint_name_list.append(
                "fr3_joint{}".format(j))
            default_robot_state_msg.joint_pos_list.append(
                -0.5 * np.pi if j == 4 else 0.0
            )

        rate = node.create_rate(30)
        start_t = rclpy.clock.Clock().now().nanoseconds / 1e9
        fail_count = 0
        fail_count_thre = 20
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

            if rosnode_ping("rviz2", max_count=1):
                fail_count = 0
            else:
                fail_count += 1
            if fail_count >= fail_count_thre:
                node.get_logger().error("rviz is not launched")
                sys.exit(1)

            if t - start_t > 10.0:
                break

            rclpy.spin_once(node, timeout_sec=0.1)
            rate.sleep()


def rosnode_ping(node_name, max_count):
    cmd = "ros2 node info {}".format(node_name)
    for i in range(max_count):
        if os.system(cmd) == 0:
            return True


if __name__ == "__main__":
    unittest.main()
