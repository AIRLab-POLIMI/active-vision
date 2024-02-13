from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    
    ld.add_action(
        DeclareLaunchArgument(
            "model_type",
            default_value="vit_b",
            choices=["vit_h", "vit_b", "vit_l"],
            description="Type of the model to use",
        )
    )
    
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
        )
    )
    return ld