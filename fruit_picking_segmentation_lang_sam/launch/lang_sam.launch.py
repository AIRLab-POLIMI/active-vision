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
            default_value="service",
            choices=["service", "pub_sub"],
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
            "image_name",
            default_value="car.jpeg",
            description="Image to segment",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "prompt",
            default_value="wheel",
            description="Text prompt to give to the LANG SAM model",
        )
    )




    # Nodes
    
    ld.add_action(
        Node(
            package="fruit_picking_segmentation_lang_sam",
            executable="lang_sam_server",
            name="lang_sam_server",
            output="screen",
            parameters=[
                {
                    "model_type": LaunchConfiguration("model_type"),
                }
            ],
            condition=LaunchConfigurationEquals('approach', 'service'),
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
                    "image_name": LaunchConfiguration("image_name"),
                    "prompt": LaunchConfiguration("prompt"),
                }
            ],
            condition=LaunchConfigurationEquals('approach', 'service'),
        )
    )

    # ld.add_action(
    #     Node(
    #         package="fruit_picking_segmentation_lang_sam",
    #         executable="lang_sam_pub_sub",
    #         name="lang_sam_pub_sub",
    #         output="screen",
    #         parameters=[
    #             {
    #                 "model_type": LaunchConfiguration("model_type"),
    #             }
    #         ],
    #         condition=LaunchConfigurationEquals('approach', 'pub_sub'),
    #     )
    # )

    return ld