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
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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

    load_rviz_arg = DeclareLaunchArgument(
        name="load_rviz",
        default_value="false",
        choices=["true", "false"],
        description="Whether or not Rviz is used",
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

    multiple_output_topics_arg = DeclareLaunchArgument(
        name="multiple_output_topics",
        default_value="true",
        choices=["true", "false"],
        description="Argument to specify the creation of multiple output topics containing each a different mask",
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
            env_gazebo_package_arg,
            full_world_name_arg,
            multiple_output_topics_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs):



    # Array of action that will be returned at the end for execution
    return_actions = []


    # Parameters

    # Sim time
    if LaunchConfiguration("load_gazebo").perform(context) == 'true':
        use_sim_time = True
    else:
        use_sim_time = False

    
    # Frames
    if LaunchConfiguration("load_gazebo").perform(context) == 'true':
        frame_id = 'world'
    else:
        frame_id = 'igus_rebel_base_link'

    
    # Data topics. Change their value from here. In the inner launch file the default value are currently the below ones
    rgb_image_topic = "/virtual_camera_link/rgbd_camera/image_raw"
    depth_image_topic = "/virtual_camera_link/rgbd_camera/depth_image"
    rgb_segmented_image_topic = "/fruit_picking/segmentation/lang_sam/image"
    camera_info_topic = "/virtual_camera_link/rgbd_camera/camera_info"
    pointcloud_processed_topic = "/fruit_picking/pointcloud/pointcloud_processed"

    octomap_occupied_cells_vis_topic = "/fruit_picking/lang_sam_segmented_octomap/occupied_cells_vis"
    octomap_free_cells_vis_topic = "/fruit_picking/lang_sam_segmented_octomap/free_cells_vis"
    octomap_occupied_cells_centers_topic = "/fruit_picking/lang_sam_segmented_octomap/occupied_cells_centers"
    octomap_binary_topic = "/fruit_picking/lang_sam_segmented_octomap/octomap_binary"
    octomap_full_topic = "/fruit_picking/lang_sam_segmented_octomap/octomap_full"
    octomap_projected_map_topic = "/fruit_picking/lang_sam_segmented_octomap/projected_map"





    # Igus Rebel description launch
    description_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("igus_rebel_description_ros2"), '/launch', '/visualize.launch.py'])
    )

    # LANG SAM segmentation launch
    lang_sam_segmentation_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("fruit_picking_segmentation_lang_sam"), '/launch', '/lang_sam.launch.py']),
        launch_arguments={
            "use_sim_time": str(use_sim_time).lower(),
            "input_image_topic": rgb_image_topic,
            "output_image_topic": rgb_segmented_image_topic,
            "multiple_output_topics": LaunchConfiguration("multiple_output_topics")
        }.items(),
    )
    

    # Pointcloud creation launch
    pointcloud_creation_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("fruit_picking_pointcloud"), '/launch', '/pointcloud_creation.launch.py']),
        launch_arguments={
            "use_sim_time": "true",
            "depth_image_topic": depth_image_topic,
            "rgb_image_topic": rgb_segmented_image_topic,
            "camera_info_topic": camera_info_topic,
            "pointcloud_processed_topic": pointcloud_processed_topic,
        }.items(),
    ) 

    # Octomap creation launch
    octomap_creation_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("fruit_picking_octomap"), '/launch', '/octomap_creation.launch.py']),
        launch_arguments={
            "use_sim_time": str(use_sim_time).lower(),
            "input_cloud_topic": pointcloud_processed_topic,
            "output_occupied_cells_vis": octomap_occupied_cells_vis_topic,
            "output_free_cells_vis": octomap_free_cells_vis_topic,
            "output_occupied_cells_centers": octomap_occupied_cells_centers_topic,
            "output_octomap_binary": octomap_binary_topic,
            "output_octomap_full": octomap_full_topic,
            "output_projected_map": octomap_projected_map_topic,
            "resolution": '0.01',
            "frame_id": frame_id,
            "height_map": "False",
            "colored_map": "True",
            "filter_ground": 'True',
        }.items(),
    ) 

    
    # Rviz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d", 
            PathJoinSubstitution([
                FindPackageShare("fruit_picking_bringup"),
                "rviz", "octomap_segmentation.rviz"
            ]),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )




    # Returns  
    return_actions.append(description_launch_file)
    return_actions.append(lang_sam_segmentation_launch_file)
    return_actions.append(pointcloud_creation_launch_file)
    # return_actions.append(octomap_creation_launch_file)
    return_actions.append(rviz_node)



    return return_actions