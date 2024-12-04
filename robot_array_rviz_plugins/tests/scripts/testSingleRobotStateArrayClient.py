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
        self.publisher_ = self.create_publisher(RobotStateArray, 'robot_state_arr', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, 
                                       self.timer_callback)
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
        t =  self.get_clock().now().to_msg().sec
        robot_state_arr_msg = RobotStateArray()

        for i in range(5):
            robot_state_msg = copy.deepcopy(self.default_robot_state_msg_ur5e)
            robot_state_msg.root_pose.pose.position.x = np.sin(0.5 * t)
            t_i = t - 0.4 * i
            robot_state_msg.joint_pos_list[0] = 2.0 * np.cos(0.5 * t_i)
            robot_state_msg.joint_pos_list[1] = 1.0 * np.sin(1.0 * t_i)
            robot_state_msg.joint_pos_list[2] = 1.0 * np.cos(1.0 * t_i)
            robot_state_arr_msg.robot_states.append(robot_state_msg)
        self.publisher_.publish(robot_state_arr_msg)
        self.get_logger().info('Publishing: "%s"' % robot_state_arr_msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    test_publisher = TestSingleRobotStatePublisher()
    rclpy.spin(test_publisher)
    test_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()