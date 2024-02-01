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
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    load_base_arg = DeclareLaunchArgument(
        name="load_base",
        default_value="true",
        description="Load the mobile robot model and tower",
        choices=["true", "false"],
    )

    mount_arg = DeclareLaunchArgument(
        name="mount",
        default_value="none",
        choices=["none", "mount_v1"],
        description="Which mount to attach to the flange",
    )

    camera_arg = DeclareLaunchArgument(
        name="camera",
        default_value="none",
        choices=["realsense", "oakd", "none"],
        description="Which camera to attach to the mount",
    )

    end_effector_arg = DeclareLaunchArgument(
        name="end_effector",
        default_value="none",
        choices=["toucher_cylinder", "none"],
        description="Which end_effector to attach to the mount",
    )

    hardware_protocol_arg = DeclareLaunchArgument(
        name="hardware_protocol",
        default_value="simulation",
        choices=["mock_hardware", "cri", "simulation", "ignition"],
        description="Which hardware protocol or mock hardware should be used",
    )    

    load_gazebo_arg = DeclareLaunchArgument(
        name="load_gazebo",
        default_value="false",
        choices=["true", "false"],
        description="Whether or not Gazebo Ignition is used",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Argument for choose to use simulation time or not",
    )

    jsp_gui_arg = DeclareLaunchArgument(
        name="jsp_gui",
        default_value="false",
        choices=["true", "false"],
        description="Argument for choose to load joint state publisher gui to send joint states in ROS",
    )

    moveit_arg = DeclareLaunchArgument(
        name="moveit",
        default_value="false",
        choices=["true", "false"],
        description="Argument for tell ROS that we are using moveit or not",
    )
    

    return LaunchDescription(
        [
            load_base_arg,
            mount_arg,
            camera_arg,
            end_effector_arg,
            hardware_protocol_arg,
            load_gazebo_arg,
            use_sim_time_arg,
            jsp_gui_arg,
            moveit_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs):



    # Array of action that will be returned at the end for execution
    return_actions = []
    


    # Sim time
    if LaunchConfiguration("use_sim_time").perform(context) == 'true' or LaunchConfiguration("load_gazebo").perform(context) == 'true':
        use_sim_time = True
    else:
        use_sim_time = False

    if LaunchConfiguration("hardware_protocol").perform(context) == 'cri':
        use_sim_time = False



    # URDF
    robot_description_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_description_ros2"),
            "urdf",
            "robot.urdf.xacro",
        ]
    )



    # RViz
    rviz_file = PathJoinSubstitution(
        [FindPackageShare("igus_rebel_description_ros2"), "rviz", "visualize.rviz"]
    )



    # Robot description
    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_file,
            " load_base:=",
            LaunchConfiguration("load_base"),
            " mount:=",
            LaunchConfiguration("mount"),
            " camera:=",
            LaunchConfiguration("camera"),
            " end_effector:=",
            LaunchConfiguration("end_effector"),
            " hardware_protocol:=",
            LaunchConfiguration("hardware_protocol"),
            " load_gazebo:=",
            LaunchConfiguration("load_gazebo"),
            " moveit:=",
            LaunchConfiguration("moveit"),
        ]
    )



    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            {'use_sim_time': use_sim_time},
        ],
    )


    if LaunchConfiguration("jsp_gui").perform(context) == 'true' or LaunchConfiguration("load_gazebo").perform(context) == 'false':
        joint_state_publisher_gui_node = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            parameters=[{'use_sim_time': use_sim_time},],
        )
        return_actions.append(joint_state_publisher_gui_node)


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
        parameters=[{'use_sim_time': use_sim_time},],
    )



    # Ignition node
    if LaunchConfiguration("load_gazebo").perform(context) == "true":

        ignition_launch_file = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("igus_rebel_gazebo_ignition"), '/launch', '/ignition.launch.py'])
            )

        return_actions.append(ignition_launch_file)



    # Returns
    return_actions.append(robot_state_publisher_node)
    return_actions.append(rviz_node)

    
    return return_actions