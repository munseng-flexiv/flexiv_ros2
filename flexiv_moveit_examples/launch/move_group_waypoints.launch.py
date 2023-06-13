import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    flexiv_urdf_xacro = os.path.join(
        get_package_share_directory("flexiv_description"), "urdf", "rizon.urdf.xacro"
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            flexiv_urdf_xacro,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt configuration
    flexiv_srdf_xacro = os.path.join(
        get_package_share_directory("flexiv_moveit_config"), "srdf", "rizon.srdf.xacro"
    )
    robot_description_semantic_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            flexiv_srdf_xacro,
            " ",
            "name:=",
            "rizon",
        ]
    )

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    kinematics_yaml = load_yaml("flexiv_moveit_config", "config/kinematics.yaml")

    # MoveGroupInterface demo executable
    move_group_waypoints = Node(
        name="move_group_waypoints",
        package="flexiv_moveit_examples",
        executable="move_group_waypoints",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml],
    )

    return LaunchDescription([move_group_waypoints])