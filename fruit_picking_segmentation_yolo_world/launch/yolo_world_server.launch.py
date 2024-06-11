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
            "frame_id",
            default_value="world",
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
            "use_sim_time",
            default_value="false",
        )
    )




    # Nodes
    

    ld.add_action(
        Node(
            package="fruit_picking_segmentation_yolo_world",
            executable="yolo_world_server",
            name="yolo_world_server",
            output="screen",
            arguments= [
                "--ros-args",
                "--log-level",
                "yolo_world_server:=debug",
            ],
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "frame_id": LaunchConfiguration("frame_id"),
                    "publish_masks_array": LaunchConfiguration("publish_masks_array"),
                    "yolo_world_model_type": LaunchConfiguration("yolo_world_model_type"),
                    "efficient_SAM_model_type": LaunchConfiguration("efficient_SAM_model_type"),
                }
            ]
        )
    )

    return ld