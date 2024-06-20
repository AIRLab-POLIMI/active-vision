# Based on the visualize_franka script licensed under the Apache license which can be found here:
# https://github.com/frankaemika/franka_ros2/blob/develop/franka_description/launch/visualize_franka.launch.py

from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue


import yaml
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        choices=["true", "false"],
        default_value="true",
        description="Parent frame of the camera",
    )

    

    return LaunchDescription(
        [
            use_sim_time_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs):


    
    # Load the configuration YAML file
    config_yaml_path = os.path.join(
        get_package_share_directory("fruit_picking_bringup"), "config/parameters.yaml")
    with open(config_yaml_path, 'r') as file:
        config_yaml = yaml.safe_load(file)



    # Array of action that will be returned at the end for execution
    return_actions = []



    # Parameters

    # Sim time
    if LaunchConfiguration("use_sim_time").perform(context) == 'true':
        use_sim_time = True
    else:
        use_sim_time = False

    use_sim_time_dict = {"use_sim_time": str(use_sim_time).lower()}



    # Robot state publisher
    robot_description_filename = "realsense_d435_camera.xacro"

    robot_description_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_description_ros2"),
            "urdf", "cameras",
            robot_description_filename,
        ]
    )

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_file,
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            {'use_sim_time': use_sim_time},
        ],
    )




    # IMU filter to create TF between odom and camera_bottom_screw_frame
    # The command is: ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args 
    #   -p use_mag:=false -p optional_imu_frame:=imu_optical_frame -r /imu/data_raw:=/camera/imu

    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick_node",
        arguments=[
            "--ros-args",
                "-p", "use_mag:=false",
                "-p", "optional_imu_frame:=realsense_optical_link",
                "-p", "fixed_frame:=container_realsense_link",
                "-p", "world_frame:=ned", # enu , ned
                "-r", "/imu/data_raw:=/camera/camera/imu"

        ],
    )



    # Realsense launch file
    realsense_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("realsense2_camera"), '/launch', '/rs_launch.py']),
        launch_arguments={
            **use_sim_time_dict,
            **config_yaml['launch']['realsense_launch'],
        }.items(),
    )

    
    
    return_actions.append(imu_filter_node)
    return_actions.append(robot_state_publisher_node)
    return_actions.append(realsense_launch_file)




    return return_actions