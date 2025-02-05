#! /usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def main(args=None):
    ur5_mappings = {
        "safety_limits": "true",
        "safety_pos_margin": "0.15",
        "safety_k_position": "20",
        "name": "ur",
        "tf_prefix": "",
        "ur_type": "ur5e",
    }

    fr3_mappings = {
        "load_gripper": "false",
        "arm_id": "fr3",
    }

    ur5_file_path = os.path.join(
        get_package_share_directory("ur_description"), "urdf", "ur.urdf.xacro"
    )

    fr3_file_path = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        "fr3",
        "fr3.urdf.xacro",
    )

    create_urdf(ur5_file_path, ur5_mappings, "ur5e.urdf")

    create_urdf(fr3_file_path, fr3_mappings, "fr3.urdf")


def create_urdf(xacro_file, mappings, output_file):
    urdf = xacro.process_file(xacro_file, mappings=mappings)

    urdf_file = urdf.toprettyxml(indent="  ")

    with open(output_file, "w") as file:
        file.write(urdf_file)

    return urdf_file


if __name__ == "__main__":
    main()
