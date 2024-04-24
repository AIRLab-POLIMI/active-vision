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
            "model_type",
            default_value="vit_b",
            choices=["vit_h", "vit_b", "vit_l"],
            description="Type of the model to use",
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
            "input_image_topic",
            default_value="/virtual_camera_link/rgbd_camera/image_raw",
            description="Topic that contains the original RGB data that needs to be segmented",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "output_image_topic",
            default_value="/fruit_picking/segmentation/lang_sam/image",
            description="Topic that contains the segmented RGB data that comes from the LANG SAM server",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "output_image_array_topic",
            default_value="/fruit_picking/segmentation/lang_sam/image_array",
            description="Topic that contains the segmented RGB array data that comes from the LANG SAM server",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "output_boxes_topic",
            default_value="/fruit_picking/segmentation/lang_sam/boxes",
            description="Topic that contains the boxes of the segmented RGB data that comes from the LANG SAM server",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "output_confidences_topic",
            default_value="/fruit_picking/segmentation/lang_sam/confidences",
            description="Topic that contains the confidences of the segmented RGB data that comes from the LANG SAM server",
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
                    "model_type": LaunchConfiguration("model_type"),
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
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "model_type": LaunchConfiguration("model_type"),
                    "input_image_topic": LaunchConfiguration("input_image_topic"),
                    "output_image_topic": LaunchConfiguration("output_image_topic"),
                    "output_image_array_topic": LaunchConfiguration("output_image_array_topic"),
                    "output_boxes_topic": LaunchConfiguration("output_boxes_topic"),
                    "output_confidences_topic": LaunchConfiguration("output_confidences_topic"),
                    "segmentation_prompt": LaunchConfiguration("segmentation_prompt"),
                }
            ],
            condition=LaunchConfigurationEquals('approach', 'pub_sub'),
        )
    )

    return ld