#! /usr/bin/env python3

import os
import sys
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String
from robot_array_msgs.msg import RobotDescription, RobotDescriptionArray


class TestRobotDescriptionMap(Node):
    def __init__(self):
        super().__init__("test_robot_description_map")

        _ = self.create_subscription(
            String, "/fr3/robot_description", self.fr3_callback, 10)

        _ = self.create_subscription(
            String, "/ur5e/robot_description", self.ur5e_callback, 10)

        self._robot_description_map_publisher = self.create_publisher(
            RobotDescriptionArray, "robot_description_map", 1)

        self._fr3_msg, self._ur5e_msg = None, None

        self._root_folder = "/root/catkin_ws/src/robot_array_rviz_plugins/" \
            "tests/urdf"

    def fr3_callback(self, msg):
        self.get_logger().info("Received fr3 robot description")
        self._fr3_msg = msg
        self.save_file(os.path.join(self._root_folder, "fr3.urdf"), msg.data)

    def ur5e_callback(self, msg):
        self.get_logger().info("Received ur5e robot description")
        self._ur5e_msg = msg
        self.save_file(os.path.join(self._root_folder, "ur5e.urdf"), msg.data)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if self._fr3_msg is None or self._ur5e_msg is None:
                continue

            self.publish_robot_description_map(
                self._fr3_msg, self._ur5e_msg)

            break

    def publish_robot_description_map(self, fr3_msg, ur5e_msg):
        self.get_logger().info("Publishing robot description map")
        robot_description_map_msg = RobotDescriptionArray()

        fr3_description = RobotDescription()

        fr3_description.name = "fr3"

        fr3_description.urdf_content = fr3_msg.data

        ur5e_description = RobotDescription()

        ur5e_description.name = "ur5e"

        ur5e_description.urdf_content = ur5e_msg.data

        robot_description_map_msg.robot_descriptions = [
            ur5e_description, fr3_description]

        self._robot_description_map_publisher.publish(
            robot_description_map_msg)

        self.save_file(os.path.join(self._root_folder, "fr3+ur5e.message"),
                       serialize_message(
                           robot_description_map_msg), mode="wb")

    def save_file(self, filename, content, mode="w"):
        with open(filename, mode) as f:
            f.write(content)


def main(args=None):
    rclpy.init(args=args)

    test_robot_description_map = TestRobotDescriptionMap()

    test_robot_description_map.run()

    test_robot_description_map.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
