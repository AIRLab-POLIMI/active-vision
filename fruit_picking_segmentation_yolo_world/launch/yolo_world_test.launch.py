from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    



    # Arguments

    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        )
    )


    # Nodes
    
    ld.add_action(
        Node(
            package="fruit_picking_segmentation_yolo_world",
            executable="yolo_world_test",
            name="yolo_world_test",
            output="screen",
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ]
        )
    )

    return ld