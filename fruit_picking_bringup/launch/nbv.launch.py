# Based on the visualize_franka script licensed under the Apache license which can be found here:
# https://github.com/frankaemika/franka_ros2/blob/develop/franka_description/launch/visualize_franka.launch.py

from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import yaml
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    load_base_arg = DeclareLaunchArgument(
        name="load_base",
        default_value="false",
        description="Load the mobile robot model and tower",
        choices=["true", "false"],
    )

    mount_arg = DeclareLaunchArgument(
        name="mount",
        default_value="mount_v1",
        choices=["none", "mount_v1"],
        description="Which mount to attach to the flange",
    )

    camera_arg = DeclareLaunchArgument(
        name="camera",
        default_value="realsense",
        choices=["realsense", "oakd", "none"],
        description="Which camera to attach to the mount",
    )

    end_effector_arg = DeclareLaunchArgument(
        name="end_effector",
        default_value="toucher_v1",
        choices=["toucher_v1", "none"],
        description="Which end_effector to attach to the mount",
    )

    hardware_protocol_arg = DeclareLaunchArgument(
        name="hardware_protocol",
        default_value="ignition",
        choices=["mock_hardware", "cri", "simulation", "ignition"],
        description="Which hardware protocol or mock hardware should be used",
    )    

    load_gazebo_arg = DeclareLaunchArgument(
        name="load_gazebo",
        default_value="true",
        choices=["true", "false"],
        description="Whether or not Gazebo Ignition is used",
    )

    env_gazebo_package_arg = DeclareLaunchArgument(
        name="env_gazebo_package",
        default_value="fruit_picking_gazebo_ignition",
        description="Package where the gazebo world and configuration are located. Default is default, but it is possible to pass the full name of the package",
    )

    full_world_name_arg = DeclareLaunchArgument(
        name="full_world_name",
        default_value="tomato_field.sdf",
        description="Name of the world to be loaded in Gazebo Ignition of the type: name.sdf",
    )

    run_robot_moveit_arg = DeclareLaunchArgument(
        name="run_robot_moveit",
        default_value="false",
        choices=["true", "false"],
        description="Whether or not run the igus rebel moveit launch",
    )

    run_nbv_pipeline_arg = DeclareLaunchArgument(
        name="run_nbv_pipeline",
        default_value="false",
        choices=["true", "false"],
        description="Whether or not run the NBV pipeline",
    )
    

    return LaunchDescription(
        [
            load_base_arg,
            mount_arg,
            camera_arg,
            end_effector_arg,
            hardware_protocol_arg,
            load_gazebo_arg,
            env_gazebo_package_arg,
            full_world_name_arg,
            run_robot_moveit_arg,
            run_nbv_pipeline_arg,
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
    if LaunchConfiguration("load_gazebo").perform(context) == 'true':
        use_sim_time = True
    else:
        use_sim_time = False

    use_sim_time_dict = {"use_sim_time": str(use_sim_time).lower()}


    
    # Frames
    if LaunchConfiguration("load_gazebo").perform(context) == 'true':
        frame_id = config_yaml['frames']['frame_id']
        base_frame_id = config_yaml['frames']['base_frame_id']
    else:
        if LaunchConfiguration("test_camera").perform(context) == 'false':
            frame_id = config_yaml['frames']['base_frame_id']
            base_frame_id = config_yaml['frames']['base_frame_id']
        else:
            if LaunchConfiguration("camera").perform(context) == 'realsense':
                frame_id = config_yaml['frames']['realsense_frame_id']
                base_frame_id = config_yaml['frames']['realsense_base_frame_id']



    # Topics
    if LaunchConfiguration("load_gazebo").perform(context) == 'false':
        if LaunchConfiguration("camera").perform(context) == 'realsense':
            rgb_image_topic = config_yaml['topics']['realsense_input_data']['rgb_image_topic']
            depth_image_topic = config_yaml['topics']['realsense_input_data']['depth_image_topic']
            depth_image_camera_info_topic = config_yaml['topics']['realsense_input_data']['depth_image_camera_info_topic']

    else:
        rgb_image_topic = config_yaml['topics']['gazebo_ignition_input_data']['rgb_image_topic']
        depth_image_topic = config_yaml['topics']['gazebo_ignition_input_data']['depth_image_topic']
        depth_image_camera_info_topic = config_yaml['topics']['gazebo_ignition_input_data']['depth_image_camera_info_topic']

    


    # Input entity
    if LaunchConfiguration("run_robot_moveit").perform(context) == 'true': 
        # Igus Rebel moveit launch
        moveit_launch_file = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("igus_rebel_moveit_config"), '/launch', '/demo.launch.py'])
        )
        return_actions.append(moveit_launch_file)

    

    # Main NBV planning pipeline
    if LaunchConfiguration("run_nbv_pipeline").perform(context) == 'true': 
        # Igus Rebel moveit launch
        nbv_pipeline_launch_file = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("fruit_picking_nbv"), '/launch', '/nbv_pipeline.launch.py']),
            launch_arguments={
                **use_sim_time_dict,
                **{
                    "rgb_image_topic": rgb_image_topic,
                    "depth_image_topic": depth_image_topic,
                    "depth_image_camera_info_topic": depth_image_camera_info_topic,
                    "frame_id": frame_id,
                    "base_frame_id": base_frame_id,
                    "centralized_architecture": config_yaml['launch']['nbv']['centralized_architecture']
                },
                **config_yaml['launch']['nbv']['nbv_pipeline_launch'],

            }.items(),
        )
        return_actions.append(nbv_pipeline_launch_file)




    return return_actions