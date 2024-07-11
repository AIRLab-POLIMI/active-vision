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
            "colors",
            default_value="red",
            choices=["red", "green", "red-green"],
            description="Desired color to filter",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        )
    )


    # Nodes
    

    ld.add_action(
        Node(
            package="fruit_picking_segmentation_color_filtering",
            executable="color_filter",
            name="color_filter",
            output="screen",
            arguments= [
                "--ros-args",
                "--log-level",
                "color_filter:=info",
            ],
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "rgb_image_topic": LaunchConfiguration("rgb_image_topic"),
                    "depth_image_topic": LaunchConfiguration("depth_image_topic"),
                    "depth_image_camera_info_topic": LaunchConfiguration("depth_image_camera_info_topic"),
                    "color_filter_rgb_image_topic": LaunchConfiguration("color_filter_rgb_image_topic"),
                    "color_filter_depth_image_topic": LaunchConfiguration("color_filter_depth_image_topic"),
                    "color_filter_depth_image_camera_info_topic": LaunchConfiguration("color_filter_depth_image_camera_info_topic"),
                    "color_filter_tf_topic": LaunchConfiguration("color_filter_tf_topic"),
                    "frame_id": LaunchConfiguration("frame_id"),
                    "publish_original_depth_image": LaunchConfiguration("publish_original_depth_image"),
                    "publish_original_depth_image_camera_info": LaunchConfiguration("publish_original_depth_image_camera_info"),
                    "publish_original_tf": LaunchConfiguration("publish_original_tf"),
                    "colors": LaunchConfiguration("colors"),
                }
            ],
        )
    )

    return ld