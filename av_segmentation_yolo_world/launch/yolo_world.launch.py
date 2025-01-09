from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    



    # Arguments


    ld.add_action(
        DeclareLaunchArgument(
            "publish_masks_array",
            default_value="True",
            choices=["True", "False"],
            description="Whether the model publish the various found masks as an array or not. If False, the topic still will be echoed since it is remapped in the segmented pointcloud launch.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "yolo_world_model_type",
            default_value="yolo_world/l",
            description="YOLO World model",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "efficient_SAM_model_type",
            default_value="l0",
            description="Efficient SAM model",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "segmentation_prompt",
            default_value="tomato",
            description="Text prompt to give to the LANG SAM model",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "confidence_threshold",
            default_value="0.001",
            description="Minimum value of confidence admitted to consider an instance",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "nms_threshold",
            default_value="0.2",
            description="Minimum value of non-maximum suppression admitted to consider an instance",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "confidence_normalization",
            default_value="false",
        )
    )




    # Nodes
    

    ld.add_action(
        Node(
            package="av_segmentation_yolo_world",
            executable="yolo_world",
            name="yolo_world",
            output="screen",
            arguments= [
                "--ros-args",
                "--log-level",
                "yolo_world:=debug",
            ],
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "rgb_image_topic": LaunchConfiguration("rgb_image_topic"),
                    "depth_image_topic": LaunchConfiguration("depth_image_topic"),
                    "depth_image_camera_info_topic": LaunchConfiguration("depth_image_camera_info_topic"),
                    "yolo_world_rgb_image_topic": LaunchConfiguration("yolo_world_rgb_image_topic"),
                    "yolo_world_rgb_images_array_topic": LaunchConfiguration("yolo_world_rgb_images_array_topic"),
                    "yolo_world_depth_image_topic": LaunchConfiguration("yolo_world_depth_image_topic"),
                    "yolo_world_depth_image_camera_info_topic": LaunchConfiguration("yolo_world_depth_image_camera_info_topic"),
                    "confidences_topic": LaunchConfiguration("confidences_topic"),
                    "yolo_world_tf_topic": LaunchConfiguration("yolo_world_tf_topic"),
                    "frame_id": LaunchConfiguration("frame_id"),
                    "publish_masks_array": LaunchConfiguration("publish_masks_array"),
                    "publish_original_depth_image": LaunchConfiguration("publish_original_depth_image"),
                    "publish_original_depth_image_camera_info": LaunchConfiguration("publish_original_depth_image_camera_info"),
                    "publish_original_tf": LaunchConfiguration("publish_original_tf"),
                    "yolo_world_model_type": LaunchConfiguration("yolo_world_model_type"),
                    "efficient_SAM_model_type": LaunchConfiguration("efficient_SAM_model_type"),
                    "segmentation_prompt": LaunchConfiguration("segmentation_prompt"),
                    "confidence_threshold": LaunchConfiguration("confidence_threshold"),
                    "nms_threshold": LaunchConfiguration("nms_threshold"),
                    "confidence_normalization": LaunchConfiguration("confidence_normalization"),
                }
            ]
        )
    )

    return ld