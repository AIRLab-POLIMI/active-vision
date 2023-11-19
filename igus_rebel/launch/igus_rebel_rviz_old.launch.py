# Based on the visualize_franka script licensed under the Apache license which can be found here:
# https://github.com/frankaemika/franka_ros2/blob/develop/franka_description/launch/visualize_franka.launch.py

from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import OpaqueFunction


def generate_launch_description():

    return LaunchDescription([OpaqueFunction(function=launch_setup),])


def launch_setup(context, *args, **kwargs):
    
    desc_file = "igus_rebel.urdf.xacro"

    robot_description_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel"),
            "urdf",
            desc_file,
        ]
    )

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_file
        ]
    )

    print("\n\n\nspace")
    print(ParameterValue(robot_description, value_type=str))
    print("\n\n\n")

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("igus_rebel"), "rviz", "igus_rebel.rviz"]
    )

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)}
        ],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file], # -d argument is for specifying the conf file of rviz
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]
