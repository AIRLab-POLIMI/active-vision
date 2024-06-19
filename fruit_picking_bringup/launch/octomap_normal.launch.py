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

    load_rviz_arg = DeclareLaunchArgument(
        name="load_rviz",
        default_value="false",
        choices=["true", "false"],
        description="Whether or not Rviz is used in the Igus Rebel description launch",
    )

    moveit_arg = DeclareLaunchArgument(
        name="moveit",
        default_value="false",
        choices=["true", "false"],
        description="Whether or not Moveit2 is used",
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

    test_camera_arg = DeclareLaunchArgument(
        name="test_camera",
        default_value="false",
        choices=["true", "false"],
        description="Whether or not only the camera is used instead of all the Igus Rebel",
    )
    

    return LaunchDescription(
        [
            load_base_arg,
            mount_arg,
            camera_arg,
            end_effector_arg,
            hardware_protocol_arg,
            load_gazebo_arg,
            load_rviz_arg,
            moveit_arg,
            env_gazebo_package_arg,
            full_world_name_arg,
            test_camera_arg,
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

    pointcloud_topic = config_yaml['topics']['pointcloud']['pointcloud_topic']

    octomap_occupied_cells_vis_topic = config_yaml['topics']['octomap']['octomap_occupied_cells_vis_topic']
    octomap_free_cells_vis_topic = config_yaml['topics']['octomap']['octomap_free_cells_vis_topic']
    octomap_occupied_cells_centers_pointcloud_topic = config_yaml['topics']['octomap']['octomap_occupied_cells_centers_pointcloud_topic']
    octomap_binary_topic = config_yaml['topics']['octomap']['octomap_binary_topic']
    octomap_full_topic = config_yaml['topics']['octomap']['octomap_full_topic']
    octomap_projected_map_topic = config_yaml['topics']['octomap']['octomap_projected_map_topic']

    # Input entity
    if LaunchConfiguration("test_camera").perform(context) == 'false': 
        # Igus Rebel description launch
        description_launch_file = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("igus_rebel_description_ros2"), '/launch', '/visualize.launch.py'])
        )
        return_actions.append(description_launch_file)
    else:
        # Realsense launch file
        realsense_launch_file = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("fruit_picking_bringup"), '/launch', '/realsense.launch.py']),
            launch_arguments={
                **use_sim_time_dict,
            }.items(),
        )
        return_actions.append(realsense_launch_file)
    

    # Pointcloud creation launch
    pointcloud_creation_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("fruit_picking_pointcloud"), '/launch', '/pointcloud_creation.launch.py']),
        launch_arguments={
            **use_sim_time_dict,
            **{
                "depth_image_topic": depth_image_topic,
                "rgb_image_topic": rgb_image_topic,
                "depth_image_camera_info_topic": depth_image_camera_info_topic,
                "pointcloud_topic": pointcloud_topic
            }
        }.items(),
    ) 


    # Extended octomap creation launch
    extended_octomap_creation_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("fruit_picking_octomap"), '/launch', '/extended_octomap_creation.launch.py']),
        launch_arguments={
            **use_sim_time_dict,
            **{
                "pointcloud_topic": pointcloud_topic,
                "octomap_occupied_cells_vis_topic": octomap_occupied_cells_vis_topic,
                "octomap_free_cells_vis_topic": octomap_free_cells_vis_topic,
                "octomap_occupied_cells_centers_pointcloud_topic": octomap_occupied_cells_centers_pointcloud_topic,
                "octomap_binary_topic": octomap_binary_topic,
                "octomap_full_topic": octomap_full_topic,
                "octomap_projected_map_topic": octomap_projected_map_topic,
                "frame_id": frame_id,
                "base_frame_id": base_frame_id
            },
            **config_yaml['launch']['octomap_normal_launch']['extended_octomap_creation_launch'],
        }.items(),
    ) 

    
    # Rviz node
    if LaunchConfiguration("load_gazebo").perform(context) == 'true':
        rviz_config_file_name = config_yaml["launch"]["octomap_normal_launch"]["rviz"]["ignition"]
    else:
        if LaunchConfiguration("test_camera").perform(context) == 'false':
            rviz_config_file_name = config_yaml["launch"]["octomap_normal_launch"]["rviz"]["real"]
        else:
            if LaunchConfiguration("camera").perform(context) == 'realsense':
                rviz_config_file_name = config_yaml["launch"]["octomap_normal_launch"]["rviz"]["realsense"]


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d", 
            PathJoinSubstitution([
                FindPackageShare("fruit_picking_bringup"),
                "rviz", rviz_config_file_name
            ]),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )






    # Returns  
    return_actions.append(pointcloud_creation_launch_file)
    return_actions.append(extended_octomap_creation_launch_file)
    return_actions.append(rviz_node)



    return return_actions