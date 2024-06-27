# Based on the visualize_franka script licensed under the Apache license which can be found here:
# https://github.com/frankaemika/franka_ros2/blob/develop/franka_description/launch/visualize_franka.launch.py

from launch import LaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
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
        default_value="mount_v2",
        choices=["none", "mount_v1", "mount_v2"],
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
        default_value="soft_gripper",
        choices=["toucher_v1", "none", "soft_gripper"],
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

    moveit_arg = DeclareLaunchArgument(
        name="moveit",
        default_value="true",
        choices=["true", "false"],
        description="Whether or not Moveit need to be executed",
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

    run_yolo_world_arg = DeclareLaunchArgument(
        name="run_yolo_world",
        default_value="false",
        choices=["true", "false"],
        description="Whether or not run the yolo world server for segmentation",
    )

    run_active_vision_pipeline_arg = DeclareLaunchArgument(
        name="run_active_vision_pipeline",
        default_value="false",
        choices=["true", "false"],
        description="Whether or not run the active vision pipeline",
    )
    

    return LaunchDescription(
        [
            load_base_arg,
            mount_arg,
            camera_arg,
            end_effector_arg,
            hardware_protocol_arg,
            load_gazebo_arg,
            moveit_arg,
            env_gazebo_package_arg,
            full_world_name_arg,
            run_robot_moveit_arg,
            run_yolo_world_arg,
            run_active_vision_pipeline_arg,
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
    if LaunchConfiguration("hardware_protocol").perform(context) == 'ignition' and LaunchConfiguration("load_gazebo").perform(context) == 'true':
        use_sim_time = True
    else:
        use_sim_time = False

    use_sim_time_dict = {"use_sim_time": str(use_sim_time).lower()}


    
    # Frames
    if LaunchConfiguration("hardware_protocol").perform(context) == 'ignition' and LaunchConfiguration("load_gazebo").perform(context) == 'true':
        frame_id = config_yaml['frames']['frame_id']
        base_frame_id = config_yaml['frames']['base_frame_id']
        camera_frame_id = config_yaml['frames']['gazebo_ignition_camera_frame_id']

    elif LaunchConfiguration("hardware_protocol").perform(context) == 'simulation' and LaunchConfiguration("load_gazebo").perform(context) == 'false':
        camera_frame_id = config_yaml['frames']['realsense_camera_frame_id']
        frame_id = config_yaml['frames']['frame_id']
        base_frame_id = config_yaml['frames']['base_frame_id']
    
    elif LaunchConfiguration("hardware_protocol").perform(context) == 'cri' and LaunchConfiguration("load_gazebo").perform(context) == 'false':
        camera_frame_id = config_yaml['frames']['realsense_camera_frame_id']
        frame_id = config_yaml['frames']['frame_id']
        base_frame_id = config_yaml['frames']['base_frame_id']




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
        moveit_launch_file = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("igus_rebel_moveit_config"), '/launch', '/demo.launch.py']),
            launch_arguments={
                "rviz_file": os.path.join(
                    get_package_share_directory("fruit_picking_bringup"), "rviz/active_vision_predefined_planning_pipeline.rviz"),
            }.items(),
        )
        return_actions.append(moveit_launch_file)




    # YOLO World server segmentation launch
    if LaunchConfiguration("run_yolo_world").perform(context) == 'true': 
        yolo_world_server_segmentation_launch_file = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("fruit_picking_segmentation_yolo_world"), '/launch', '/yolo_world_server.launch.py']),
            launch_arguments={
                **use_sim_time_dict,
                **{
                    "frame_id": frame_id,
                },
                **config_yaml['launch']['active_vision_pipeline_launch']['yolo_world_server']

            }.items(),
        )
        return_actions.append(yolo_world_server_segmentation_launch_file)

    

    # Active vision pipeline
    if LaunchConfiguration("run_active_vision_pipeline").perform(context) == 'true': 
        predefined_planning_pipeline_launch_file = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("fruit_picking_planning"), '/launch', '/predefined_planning_pipeline.launch.py']),
            launch_arguments={
                **use_sim_time_dict,
                **{
                    "rgb_image_topic": rgb_image_topic,
                    "depth_image_topic": depth_image_topic,
                    "depth_image_camera_info_topic": depth_image_camera_info_topic,
                    "frame_id": frame_id,
                    "base_frame_id": base_frame_id,
                    "camera_frame": camera_frame_id,
                    "centralized_architecture": config_yaml['launch']['active_vision_pipeline_launch']['centralized_architecture']
                },
                **config_yaml['launch']['active_vision_pipeline_launch']['predefined_planning_pipeline_launch'],

            }.items(),
        )
        return_actions.append(predefined_planning_pipeline_launch_file)




    return return_actions