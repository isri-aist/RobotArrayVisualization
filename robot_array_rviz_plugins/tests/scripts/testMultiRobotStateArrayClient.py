#! /usr/bin/env python

import sys
import unittest
import copy
import numpy as np
import rospy
import rostest
import rosnode
from geometry_msgs.msg import PoseStamped
from robot_array_msgs.msg import RobotState, RobotStateArray


class TestMultiRobotStateArrayClient(unittest.TestCase):
    def __init__(self, *args):
        super().__init__(*args)

    def test(self):
        rospy.init_node("client")

        pub = rospy.Publisher("robot_state_arr", RobotStateArray, queue_size=1)

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
        default_robot_state_msg_panda.name = "panda"
        default_robot_state_msg_panda.root_pose = default_pose_st_msg
        panda_joint_name_list = ["panda_joint{}".format(i) for i in range(1, 8)]
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

        rate = rospy.Rate(30)
        start_t = rospy.get_time()
        fail_count = 0
        fail_count_thre = 20
        while not rospy.is_shutdown():
            t = rospy.get_time()

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

            if rosnode.rosnode_ping("rviz", max_count=1):
                fail_count = 0
            else:
                fail_count += 1
            if fail_count >= fail_count_thre:
                rospy.logerr("Ping to rviz failed")
                sys.exit(1)

            if t - start_t > 10.0:
                break

            rate.sleep()


if __name__ == "__main__":
    rostest.rosrun(
        "robot_array_rviz_plugins",
        "TestMultiRobotStateArrayDisplay",
        TestMultiRobotStateArrayClient,
        sys.argv,
    )
