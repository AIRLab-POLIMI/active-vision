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
            "input_image_topic",
            default_value="/virtual_camera_link/rgbd_camera/image_raw",
            description="Topic that contains the original RGB data that needs to be segmented",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "output_image_topic",
            default_value="/fruit_picking/segmentation/color_filter/image",
            description="Topic that contains the filtered RGB data that comes from the color filter",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "colors",
            default_value="red",
            choices=["red", "green", "red-green"],
            description="Desired color to filter",
        )
    )




    # Nodes
    

    ld.add_action(
        Node(
            package="fruit_picking_segmentation_color_filtering",
            executable="color_filter",
            name="color_filter",
            output="screen",
            parameters=[
                {
                    "input_image_topic": LaunchConfiguration("input_image_topic"),
                    "output_image_topic": LaunchConfiguration("output_image_topic"),
                    "colors": LaunchConfiguration("colors"),
                }
            ],
        )
    )

    return ld