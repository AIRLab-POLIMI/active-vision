from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    



    # Arguments

    ld.add_action(
        DeclareLaunchArgument(
            "approach",
            default_value="pub_sub",
            choices=["test_client", "pub_sub"],
            description="Approach of the LANG SAM model.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "sam_model_type",
            default_value="vit_b",
            choices=["vit_h", "vit_b", "vit_l"],
            description="Type of the SAM model to use",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "test_image_name",
            default_value="car.jpeg",
            description="Image to segment for the test LANG SAM model",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "test_prompt",
            default_value="wheel",
            description="Text prompt to give to the test LANG SAM model",
        )
    )

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
            "segmentation_prompt",
            default_value="tomato",
            description="Text prompt to give to the LANG SAM model",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        )
    )




    # Nodes
    
    # Server is always executed
    ld.add_action(
        Node(
            package="fruit_picking_segmentation_lang_sam",
            executable="lang_sam_server",
            name="lang_sam_server",
            output="screen",
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "sam_model_type": LaunchConfiguration("sam_model_type"),
                }
            ],
            condition=LaunchConfigurationEquals('approach', 'test_client'),
        )
    )

    ld.add_action(
        Node(
            package="fruit_picking_segmentation_lang_sam",
            executable="lang_sam_client",
            name="lang_sam_client",
            output="screen",
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "test_image_name": LaunchConfiguration("test_image_name"),
                    "test_prompt": LaunchConfiguration("test_prompt"),
                }
            ],
            condition=LaunchConfigurationEquals('approach', 'test_client'),
        )
    )

    ld.add_action(
        Node(
            package="fruit_picking_segmentation_lang_sam",
            executable="lang_sam_pub_sub",
            name="lang_sam_pub_sub",
            output="screen",
            arguments= [
                "--ros-args",
                "--log-level",
                "lang_sam_pub_sub:=debug",
            ],
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "sam_model_type": LaunchConfiguration("sam_model_type"),
                    "rgb_image_topic": LaunchConfiguration("rgb_image_topic"),
                    "depth_image_topic": LaunchConfiguration("depth_image_topic"),
                    "depth_image_camera_info_topic": LaunchConfiguration("depth_image_camera_info_topic"),
                    "lang_sam_rgb_image_topic": LaunchConfiguration("lang_sam_rgb_image_topic"),
                    "lang_sam_rgb_images_array_topic": LaunchConfiguration("lang_sam_rgb_images_array_topic"),
                    "lang_sam_depth_image_topic": LaunchConfiguration("lang_sam_depth_image_topic"),
                    "lang_sam_depth_image_camera_info_topic": LaunchConfiguration("lang_sam_depth_image_camera_info_topic"),
                    "confidences_topic": LaunchConfiguration("confidences_topic"),
                    "lang_sam_tf_topic": LaunchConfiguration("lang_sam_tf_topic"),
                    "frame_id": LaunchConfiguration("frame_id"),
                    "publish_masks_array": LaunchConfiguration("publish_masks_array"),
                    "publish_original_depth_image": LaunchConfiguration("publish_original_depth_image"),
                    "publish_original_depth_image_camera_info": LaunchConfiguration("publish_original_depth_image_camera_info"),
                    "publish_original_tf": LaunchConfiguration("publish_original_tf"),
                    "segmentation_prompt": LaunchConfiguration("segmentation_prompt"),
                }
            ],
            condition=LaunchConfigurationEquals('approach', 'pub_sub'),
        )
    )

    return ld