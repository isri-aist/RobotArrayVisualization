#!/usr/bin/env python3
import copy
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robot_array_msgs.msg import RobotState, RobotStateArray

class TestSingleRobotStatePublisher(Node):
    def __init__(self):
        super().__init__('TestSingleRobotStatePublisher')
        self.publisher_ = self.create_publisher(RobotStateArray, 'robot_state_arr')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.default_robot_state_msg_ur5e = RobotState()
        self.default_robot_state_msg_ur5e.name = "ur5e"
        self.default_robot_state_msg_ur5e.root_pose.header.frame_id = "map"
        self.default_robot_state_msg_ur5e.root_pose.pose.position.x = 0.0
        self.default_robot_state_msg_ur5e.root_pose.pose.position.y = 0.0
        self.default_robot_state_msg_ur5e.root_pose.pose.position.z = 0.0
        self.default_robot_state_msg_ur5e.root_pose.pose.orientation.x = 0.0
        self.default_robot_state_msg_ur5e.root_pose.pose.orientation.y = 0.0
        self.default_robot_state_msg_ur5e.root_pose.pose.orientation.z = 0.0
        self.default_robot_state_msg_ur5e.root_pose.pose.orientation.w = 1.0
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
            self.default_robot_state_msg_ur5e.joint_name_list.append(jname)
            self.default_robot_state_msg_ur5e.joint_pos_list.append(jpos)

    def timer_callback(self):
        t =  self.get_clock().now()
        robot_state_arr_msg = RobotStateArray()

        for i in range(5):
            robot_state_msg = copy.deepcopy(self.default_robot_state_msg)
            robot_state_msg.root_pose.pose.position.x = np.sin(0.5 * t)
            t_i = t - 0.4 * i
            robot_state_msg.joint_pos_list[0] = 2.0 * np.cos(0.5 * t_i)
            robot_state_msg.joint_pos_list[1] = 1.0 * np.sin(1.0 * t_i)
            robot_state_msg.joint_pos_list[2] = 1.0 * np.cos(1.0 * t_i)
            robot_state_arr_msg.robot_states.append(robot_state_msg)
        self.publisher_.publish(robot_state_arr_msg)
        self.get_logger().info('Publishing: "%s"' % robot_state_arr_msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    test_publisher = TestSingleRobotStatePublisher()
    rclpy.spin(test_publisher)
    test_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# #! /usr/bin/env python

# import sys
# import unittest
# import copy
# import numpy as np
# import rospy
# import rostest
# import rosnode
# from robot_array_msgs.msg import RobotState, RobotStateArray


# class TestSingleRobotStateArrayClient(unittest.TestCase):
#     def __init__(self, *args):
#         super().__init__(*args)

#     def test(self):
#         rospy.init_node("client")

#         pub = rospy.Publisher("robot_state_arr", RobotStateArray, queue_size=1)

#         default_robot_state_msg = RobotState()
#         default_robot_state_msg.name = "panda"
#         default_robot_state_msg.root_pose.header.frame_id = "map"
#         default_robot_state_msg.root_pose.pose.position.x = 0.0
#         default_robot_state_msg.root_pose.pose.position.y = 0.0
#         default_robot_state_msg.root_pose.pose.position.z = 0.0
#         default_robot_state_msg.root_pose.pose.orientation.x = 0.0
#         default_robot_state_msg.root_pose.pose.orientation.y = 0.0
#         default_robot_state_msg.root_pose.pose.orientation.z = 0.0
#         default_robot_state_msg.root_pose.pose.orientation.w = 1.0
#         for j in range(1, 8):
#             default_robot_state_msg.joint_name_list.append("panda_joint{}".format(j))
#             default_robot_state_msg.joint_pos_list.append(
#                 -0.5 * np.pi if j == 4 else 0.0
#             )

#         rate = rospy.Rate(30)
#         start_t = rospy.get_time()
#         fail_count = 0
#         fail_count_thre = 20
#         while not rospy.is_shutdown():
#             t = rospy.get_time()

#             robot_state_arr_msg = RobotStateArray()

#             for i in range(5):
#                 robot_state_msg = copy.deepcopy(default_robot_state_msg)
#                 robot_state_msg.root_pose.pose.position.x = np.sin(0.5 * t)
#                 t_i = t - 0.4 * i
#                 robot_state_msg.joint_pos_list[0] = 2.0 * np.cos(0.5 * t_i)
#                 robot_state_msg.joint_pos_list[1] = 1.0 * np.sin(1.0 * t_i)
#                 robot_state_msg.joint_pos_list[2] = 1.0 * np.cos(1.0 * t_i)
#                 robot_state_arr_msg.robot_states.append(robot_state_msg)

#             pub.publish(robot_state_arr_msg)

#             if rosnode.rosnode_ping("rviz", max_count=1):
#                 fail_count = 0
#             else:
#                 fail_count += 1
#             if fail_count >= fail_count_thre:
#                 rospy.logerr("Ping to rviz failed")
#                 sys.exit(1)

#             if t - start_t > 10.0:
#                 break

#             rate.sleep()


# if __name__ == "__main__":
#     rostest.rosrun(
#         "robot_array_rviz_plugins",
#         "TestSingleRobotStateArrayDisplay",
#         TestSingleRobotStateArrayClient,
#         sys.argv,
#     )
