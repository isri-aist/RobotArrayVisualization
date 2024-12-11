#! /usr/bin/env python

import sys
import unittest
import json
import rclpy
from std_msgs.msg import String
from message_filters import Subscriber, ApproximateTimeSynchronizer


class TestRobotDescriptionMap(unittest.TestCase):
    def __init__(self, *args):
        super().__init__(*args)

    def test(self):
        rclpy.init(args=sys.argv)
        node = rclpy.create_node("test_robot_description_map")

        self._fr3_subscriber = Subscriber(
            node, String, "/fr3/robot_description")

        self._ur5e_subscriber = Subscriber(
            node, String, "/ur5e/robot_description")

        self._msg_synchronizer = ApproximateTimeSynchronizer(
            [self._fr3_subscriber, self._ur5e_subscriber, 10, 0.001])

        self._robot_description_map_publisher = node.create_publisher(
            String, "robot_description_map", 1)

        self._msg_synchronizer.registerCallback(self.callback)

        rclpy.spin(node)

        node.destroy_node()

        rclpy.shutdown()

    def callback(self, fr3_msg, ur5e_msg):
        print("fr3: ", fr3_msg.data)
        print("ur5e: ", ur5e_msg.data)

        robot_description_map_msg = String()

        robot_description_map = {
            "fr3": fr3_msg.data,
            "ur5e": ur5e_msg.data
        }

        robot_description_map_msg.data = json.dumps(robot_description_map)

        self._robot_description_map_publisher.publish(
            robot_description_map_msg)


if __name__ == "__main__":
    unittest.main()
