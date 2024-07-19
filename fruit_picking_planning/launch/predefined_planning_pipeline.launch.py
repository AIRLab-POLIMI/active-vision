# Based on the visualize_franka script licensed under the Apache license which can be found here:
# https://github.com/frankaemika/franka_ros2/blob/develop/franka_description/launch/visualize_franka.launch.py

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from moveit_launch import moveit_loader

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('predefined_planning', default_value='zig_zag_curve_waypoints'),
        DeclareLaunchArgument('camera_frame', default_value='igus_rebel/link_8/depth_camera'), # for moveit2_api
        DeclareLaunchArgument('load_base', default_value='False'), # for moveit2_api
        DeclareLaunchArgument('segmentation_prompt', default_value='tomato'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.001'),
        DeclareLaunchArgument('nms_threshold', default_value='0.2'),
        DeclareLaunchArgument('resolution', default_value='0.02'),
        DeclareLaunchArgument('frame_id', default_value='igus_rebel_base_link'),
        DeclareLaunchArgument('base_frame_id', default_value='igus_rebel_base_link'),
        DeclareLaunchArgument('height_map', default_value='False'),
        DeclareLaunchArgument('colored_map', default_value='False'),
        DeclareLaunchArgument('color_factor', default_value='0.8'),
        DeclareLaunchArgument('filter_ground', default_value='False'),
        DeclareLaunchArgument('filter_speckles', default_value='False'),
        DeclareLaunchArgument('ground_filter/distance', default_value='0.04'),
        DeclareLaunchArgument('ground_filter/angle', default_value='0.15'),
        DeclareLaunchArgument('ground_filter/plane_distance', default_value='0.07'),
        DeclareLaunchArgument('compress_map', default_value='True'),
        DeclareLaunchArgument('incremental_2D_projection', default_value='False'),
        DeclareLaunchArgument('sensor_model/max_range', default_value='-1.0'),
        DeclareLaunchArgument('sensor_model/hit', default_value='0.7'),
        DeclareLaunchArgument('sensor_model/miss', default_value='0.4'),
        DeclareLaunchArgument('sensor_model/min', default_value='0.12'),
        DeclareLaunchArgument('sensor_model/max', default_value='0.97'),
        DeclareLaunchArgument('pointcloud_min_x', default_value=str(float('-inf'))),  # Approximation of -std::numeric_limits<double>::max()
        DeclareLaunchArgument('pointcloud_max_x', default_value=str(float('inf'))),  # Approximation of std::numeric_limits<double>::max()
        DeclareLaunchArgument('pointcloud_min_y', default_value=str(float('-inf'))),  # Approximation of -std::numeric_limits<double>::max()
        DeclareLaunchArgument('pointcloud_max_y', default_value=str(float('inf'))), # Approximation of std::numeric_limits<double>::max()
        DeclareLaunchArgument('pointcloud_min_z', default_value=str(float('-inf'))),  # Approximation of -std::numeric_limits<double>::max()
        DeclareLaunchArgument('pointcloud_max_z', default_value=str(float('inf'))),  # Approximation of std::numeric_limits<double>::max()
        DeclareLaunchArgument('color/r', default_value='1.0'),
        DeclareLaunchArgument('color/g', default_value='1.0'),
        DeclareLaunchArgument('color/b', default_value='1.0'),
        DeclareLaunchArgument('color/a', default_value='1.0'),
        DeclareLaunchArgument('color_free/r', default_value='0.0'),
        DeclareLaunchArgument('color_free/g', default_value='0.0'),
        DeclareLaunchArgument('color_free/b', default_value='1.0'),
        DeclareLaunchArgument('color_free/a', default_value='1.0'),
        DeclareLaunchArgument('process_free_space', default_value='False'),
        DeclareLaunchArgument('publish_free_space', default_value='False'),
        DeclareLaunchArgument('publish_free_cells', default_value='False'),
        DeclareLaunchArgument('publish_octomap_binary', default_value='False'),
        DeclareLaunchArgument('publish_octomap_full', default_value='False'),
        DeclareLaunchArgument('publish_centers_pointcloud', default_value='False'),
        DeclareLaunchArgument('publish_2d_projected_map', default_value='False'),
        DeclareLaunchArgument('publish_semantic', default_value='False'),
        DeclareLaunchArgument('publish_confidence', default_value='False'),
        DeclareLaunchArgument('publish_instances', default_value='False'),
        DeclareLaunchArgument('partial_pointcloud_subscription', default_value='False'),
        DeclareLaunchArgument('segmented_pointcloud_subscription', default_value='False'),
        DeclareLaunchArgument('segmented_pointclouds_array_subscription', default_value='False'),
        DeclareLaunchArgument('message_filter_queue', default_value='5'),
        DeclareLaunchArgument('insert_cloud_init', default_value='True'),
        DeclareLaunchArgument('insert_segmented_init', default_value='True'),
        DeclareLaunchArgument('centralized_architecture', default_value='False'),
        DeclareLaunchArgument('segmented_pointcloud_outlier_removal', default_value='True'),
        DeclareLaunchArgument('use_frequency_threshold', default_value='True'),
        DeclareLaunchArgument('frequency_threshold', default_value='0.50'),
        DeclareLaunchArgument('outlier_detection', default_value='False'),
        DeclareLaunchArgument('search_neighboorhood_ray', default_value='5'),
        DeclareLaunchArgument('correction_neighboorhood_ray', default_value='5'),
        DeclareLaunchArgument('outlier_threshold', default_value='0.2'),
        DeclareLaunchArgument('weighted_confidence', default_value='False'),



    # Predefined planning pipeline node
    Node(
        package='fruit_picking_planning',
        executable='main_predefined_planning_pipeline',
        output='screen',
        arguments= [
            "--ros-args",
            "--log-level",
            "main_predefined_planning_pipeline:=info",
        ],
        remappings=[('rgb_image', LaunchConfiguration('rgb_image_topic')),
                    ('depth_image', LaunchConfiguration('depth_image_topic')),
                    ('depth_image_camera_info', LaunchConfiguration('depth_image_camera_info_topic')),
                    ('occupied_cells_vis_array', 'visualization/occupied_cells_vis_array'),
                    ('free_cells_vis_array', 'visualization/free_cells_vis_array'),
                    ('confidence_cells_vis', 'visualization/confidence_cells_vis'),
                    ('semantic_class_cells_vis', 'visualization/semantic_class_cells_vis'),
                    ('instances_cells_vis', 'visualization/instances_cells_vis'),
                ],
        parameters=moveit_loader.load_moveit(with_sensors3d=False) + [
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
            "predefined_planning": LaunchConfiguration("predefined_planning"),
            "camera_frame": LaunchConfiguration("camera_frame"),
            "load_base": LaunchConfiguration("load_base"),
            "segmentation_prompt": LaunchConfiguration("segmentation_prompt"),
            "confidence_threshold": LaunchConfiguration("confidence_threshold"),
            "nms_threshold": LaunchConfiguration("nms_threshold"),
            'resolution': LaunchConfiguration('resolution'),
            'frame_id': LaunchConfiguration('frame_id'),
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'height_map': LaunchConfiguration('height_map'),
            'colored_map': LaunchConfiguration('colored_map'),
            'color_factor': LaunchConfiguration('color_factor'),
            'filter_ground': LaunchConfiguration('filter_ground'),
            'filter_speckles': LaunchConfiguration('filter_speckles'),
            'ground_filter/distance': LaunchConfiguration('ground_filter/distance'),
            'ground_filter/angle': LaunchConfiguration('ground_filter/angle'),
            'ground_filter/plane_distance': LaunchConfiguration('ground_filter/plane_distance'),
            'compress_map': LaunchConfiguration('compress_map'),
            'incremental_2D_projection': LaunchConfiguration('incremental_2D_projection'),
            'sensor_model/max_range': LaunchConfiguration('sensor_model/max_range'),
            'sensor_model/hit': LaunchConfiguration('sensor_model/hit'),
            'sensor_model/miss': LaunchConfiguration('sensor_model/miss'),
            'sensor_model/min': LaunchConfiguration('sensor_model/min'),
            'sensor_model/max': LaunchConfiguration('sensor_model/max'),
            'pointcloud_min_x': LaunchConfiguration('pointcloud_min_x'),
            'pointcloud_max_x': LaunchConfiguration('pointcloud_max_x'),
            'pointcloud_min_y': LaunchConfiguration('pointcloud_min_y'),
            'pointcloud_max_y': LaunchConfiguration('pointcloud_max_y'),
            'pointcloud_min_z': LaunchConfiguration('pointcloud_min_z'),
            'pointcloud_max_z': LaunchConfiguration('pointcloud_max_z'),
            'color/r': LaunchConfiguration('color/r'),
            'color/g': LaunchConfiguration('color/g'),
            'color/b': LaunchConfiguration('color/b'),
            'color/a': LaunchConfiguration('color/a'),
            'color_free/r': LaunchConfiguration('color_free/r'),
            'color_free/g': LaunchConfiguration('color_free/g'),
            'color_free/b': LaunchConfiguration('color_free/b'),
            'color_free/a': LaunchConfiguration('color_free/a'),
            'process_free_space': LaunchConfiguration('process_free_space'),
            'publish_free_space': LaunchConfiguration('publish_free_space'),
            'publish_free_cells': LaunchConfiguration('publish_free_cells'),
            'publish_octomap_binary': LaunchConfiguration('publish_octomap_binary'),
            'publish_octomap_full': LaunchConfiguration('publish_octomap_full'),
            'publish_centers_pointcloud': LaunchConfiguration('publish_centers_pointcloud'),
            'publish_2d_projected_map': LaunchConfiguration('publish_2d_projected_map'),
            'publish_semantic': LaunchConfiguration('publish_semantic'),
            'publish_confidence': LaunchConfiguration('publish_confidence'),
            'publish_instances': LaunchConfiguration('publish_instances'),
            'partial_pointcloud_subscription': LaunchConfiguration('partial_pointcloud_subscription'),
            'segmented_pointcloud_subscription': LaunchConfiguration('segmented_pointcloud_subscription'),
            'segmented_pointclouds_array_subscription': LaunchConfiguration('segmented_pointclouds_array_subscription'),
            'message_filter_queue': LaunchConfiguration('message_filter_queue'),
            'insert_cloud_init': LaunchConfiguration('insert_cloud_init'),
            'insert_segmented_init': LaunchConfiguration('insert_segmented_init'),
            'centralized_architecture': LaunchConfiguration('centralized_architecture'),
            'segmented_pointcloud_outlier_removal': LaunchConfiguration('segmented_pointcloud_outlier_removal'),
            'use_frequency_threshold': LaunchConfiguration('use_frequency_threshold'),
            'frequency_threshold': LaunchConfiguration('frequency_threshold'),
            'outlier_detection': LaunchConfiguration('outlier_detection'),
            'search_neighboorhood_ray': LaunchConfiguration('search_neighboorhood_ray'),
            'correction_neighboorhood_ray': LaunchConfiguration('correction_neighboorhood_ray'),
            'outlier_threshold': LaunchConfiguration('outlier_threshold'),
            'weighted_confidence': LaunchConfiguration('weighted_confidence'),
            }]
        )
    ])

    



