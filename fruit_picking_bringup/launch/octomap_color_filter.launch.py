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
        description="Whether or not Rviz is used in the Igus Rebel description launch",
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

    color_filter_arg = DeclareLaunchArgument(
        name= "colors",
        default_value="red",
        choices=["red", "green", "red-green"],
        description="Desired color to filter",
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
            env_gazebo_package_arg,
            full_world_name_arg,
            color_filter_arg,
            test_camera_arg,
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
        base_frame_id = 'igus_rebel_base_link'
    else:
        if LaunchConfiguration("test_camera").perform(context) == 'false':
            frame_id = 'igus_rebel_base_link'
            base_frame_id = 'igus_rebel_base_link'
        else:
            if LaunchConfiguration("camera").perform(context) == 'realsense':
                frame_id = 'camera_color_optical_frame'
                base_frame_id = 'camera_color_optical_frame'

    
    # Data topics. Change their value from here. In the inner launch file the default value are currently the below ones
    if LaunchConfiguration("load_gazebo").perform(context) == 'false':
        if LaunchConfiguration("camera").perform(context) == 'realsense':
            rgb_image_topic = "/camera/color/image_raw"
            depth_image_topic = "/camera/aligned_depth_to_color/image_raw"
            depth_image_camera_info_topic = "/camera/aligned_depth_to_color/camera_info"

    else:
        rgb_image_topic = "/virtual_camera_link/rgbd_camera/image_raw"
        depth_image_topic = "/virtual_camera_link/rgbd_camera/depth_image"
        depth_image_camera_info_topic = "/virtual_camera_link/rgbd_camera/camera_info"

    color_filter_rgb_image_topic = "/fruit_picking/segmentation/color_filter/rgb_image"
    
    pointcloud_topic = "/fruit_picking/pointcloud/pointcloud"
    segmented_pointcloud_topic = "/fruit_picking/pointcloud/segmented_pointcloud"


    octomap_occupied_cells_vis_topic = "/fruit_picking/octomap/occupied_cells_vis"
    octomap_free_cells_vis_topic = "/fruit_picking/octomap/free_cells_vis"
    octomap_occupied_cells_centers_pointcloud_topic = "/fruit_picking/octomap/occupied_cells_centers_pointcloud"
    octomap_binary_topic = "/fruit_picking/octomap/octomap_binary"
    octomap_full_topic = "/fruit_picking/octomap/octomap_full"
    octomap_projected_map_topic = "/fruit_picking/octomap/projected_map"
    octomap_semantic_class_cells_vis_topic = "/fruit_picking/octomap/color_filter/semantic_class_cells_vis"




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
                FindPackageShare("realsense2_camera"), '/launch', '/rs_launch.py']),
            launch_arguments={
                "enable_rgbd": "true",
                "enable_sync": "true",
                "align_depth.enable": "true",
                "enable_color": "true",
                "enable_depth": "true",
            }.items(),
        )
        return_actions.append(realsense_launch_file)

    




    # Color filter segmentation launch
    color_filter_segmentation_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("fruit_picking_segmentation_color_filtering"), '/launch', '/color_filter.launch.py']),
        launch_arguments={
            "use_sim_time": str(use_sim_time).lower(),
            "rgb_image_topic": rgb_image_topic,
            "color_filter_rgb_image_topic": color_filter_rgb_image_topic,
            "colors": LaunchConfiguration("colors")
        }.items(),
    )
    

    # Pointcloud creation launch
    pointcloud_creation_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("fruit_picking_pointcloud"), '/launch', '/pointcloud_creation.launch.py']),
        launch_arguments={
            "use_sim_time": str(use_sim_time).lower(),
            "depth_image_topic": depth_image_topic,
            "rgb_image_topic": rgb_image_topic,
            "depth_image_camera_info_topic": depth_image_camera_info_topic,
            "pointcloud_topic": pointcloud_topic,
        }.items(),
    ) 


    # Reduced pointcloud creation launch
    segmented_pointcloud_creation_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("fruit_picking_pointcloud"), '/launch', '/segmented_pointcloud_creation.launch.py']),
        launch_arguments={
            "use_sim_time": str(use_sim_time).lower(),
            "rgb_image_topic": color_filter_rgb_image_topic,
            "depth_image_topic": depth_image_topic,
            "depth_image_camera_info_topic": depth_image_camera_info_topic,
            "segmented_pointcloud_topic": segmented_pointcloud_topic,
            "publish_pointclouds_array": "False",
            "publish_single_pointcloud": "True",
        }.items(),
    ) 


    # Octomap creation launch
    octomap_creation_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("fruit_picking_octomap"), '/launch', '/octomap_creation.launch.py']),
        launch_arguments={
            "use_sim_time": str(use_sim_time).lower(),
            "pointcloud_topic": pointcloud_topic,
            "segmented_pointcloud_topic": segmented_pointcloud_topic,
            "octomap_occupied_cells_vis_topic": octomap_occupied_cells_vis_topic,
            "octomap_free_cells_vis_topic": octomap_free_cells_vis_topic,
            "octomap_occupied_cells_centers_pointcloud_topic": octomap_occupied_cells_centers_pointcloud_topic,
            "octomap_binary_topic": octomap_binary_topic,
            "octomap_full_topic": octomap_full_topic,
            "octomap_projected_map_topic": octomap_projected_map_topic,
            "octomap_semantic_class_cells_vis_topic": octomap_semantic_class_cells_vis_topic,
            "resolution": '0.01',
            "base_frame_id": base_frame_id,
            "frame_id": frame_id,
            "height_map": "False",
            "colored_map": "False",
            "filter_ground": 'True',
            "publish_confidence": 'False',
            "publish_semantic": 'True',
            "pointcloud_array_subscription": 'False',
        }.items(),
    ) 

    
    # Rviz node
    if LaunchConfiguration("load_gazebo").perform(context) == 'true':
        rviz_config_file_name = 'octomap_color_filter_ignition.rviz'
    else:
        if LaunchConfiguration("test_camera").perform(context) == 'false':
            rviz_config_file_name = 'octomap_color_filter.rviz'
        else:
            if LaunchConfiguration("camera").perform(context) == 'realsense':
                rviz_config_file_name = 'octomap_color_filter_realsense.rviz'


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
    return_actions.append(color_filter_segmentation_launch_file)
    return_actions.append(pointcloud_creation_launch_file)
    return_actions.append(segmented_pointcloud_creation_launch_file)
    return_actions.append(octomap_creation_launch_file)
    return_actions.append(rviz_node)



    return return_actions