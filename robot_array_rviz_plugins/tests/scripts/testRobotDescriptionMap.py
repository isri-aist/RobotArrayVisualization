#! /usr/bin/env python3

import os
import sys
import time
import xacro
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from robot_array_msgs.msg import RobotDescription, RobotDescriptionArray


class TestRobotDescriptionMap(Node):
    def __init__(self):
        super().__init__("test_robot_description_map")

        ur5_mappings = {
            "safety_limits": "true",
            "safety_pos_margin": "0.15",
            "safety_k_position": "20",
            "name": "ur",
            "tf_prefix": "",
            "ur_type": "ur5e"
        }

        fr3_mappings = {
            "hand": "true",
            "ee_id": "franka_hand"
        }

        ur5_file_path = os.path.join(
            get_package_share_directory("ur_description"),
            "urdf",
            "ur.urdf.xacro"
        )

        fr3_file_path = os.path.join(
            get_package_share_directory("franka_description"),
            "robots",
            "fr3",
            "fr3.urdf.xacro"
        )

        self._ur5e_description = self.create_urdf(
            ur5_file_path, ur5_mappings)

        self._fr3_description = self.create_urdf(
            fr3_file_path, fr3_mappings)

        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._robot_description_map_publisher = self.create_publisher(
            RobotDescriptionArray, "robot_description_map",
            qos_profile=qos_profile)
        
        time.sleep(0.5)

        self.publish_robot_description_map()

    def create_urdf(self, xacro_file, mappings):
        urdf = xacro.process_file(
            xacro_file, mappings=mappings)

        urdf_file = urdf.toprettyxml(indent="  ")

        return urdf_file        

    def publish_robot_description_map(self):
        self.get_logger().info("Publishing robot description map")
        robot_description_map_msg = RobotDescriptionArray()

        fr3_description = RobotDescription()

        fr3_description.name = "fr3"

        fr3_description.urdf_content = self._fr3_description

        ur5e_description = RobotDescription()

        ur5e_description.name = "ur5e"

        ur5e_description.urdf_content = self._ur5e_description

        robot_description_map_msg.robot_descriptions = [
            ur5e_description, fr3_description]

        self._robot_description_map_publisher.publish(
            robot_description_map_msg)


def main(args=None):
    rclpy.init(args=args)

    test_robot_description_map = TestRobotDescriptionMap()

    test_robot_description_map.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
