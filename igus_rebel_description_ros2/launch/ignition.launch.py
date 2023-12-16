# Based on the visualize_franka script licensed under the Apache license which can be found here:
# https://github.com/frankaemika/franka_ros2/blob/develop/franka_description/launch/visualize_franka.launch.py

import os
from os import environ
from launch import LaunchDescription

from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
	
    ignition_models_path = os.path.join(
        get_package_share_directory("igus_rebel_description_ros2"), "models",
    )
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = ignition_models_path

    env = {  # IGN GAZEBO FORTRESS env variables
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": ":".join(
            [
                environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", default=""),
                environ.get("LD_LIBRARY_PATH", default=""),
            ]
        ),
    }

    world_path = os.path.join(
        get_package_share_directory("igus_rebel_description_ros2"), "worlds", "tomato_field_big.sdf",
    )

    gazebo_config_gui_path = os.path.join(
        get_package_share_directory("igus_rebel_description_ros2"),
        "config",
        "gazebo_gui.config",
    )

    ign_sim = ExecuteProcess(
        cmd=[
            "ign gazebo",
            "--verbose 1 -r --gui-config " + gazebo_config_gui_path,
            world_path,
        ],
        output="log",
        additional_env=env,
        shell=True,
    )

    ign_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "igus_rebel",
            "-z", '0.0',
            "-x", '-2.0',
            "-y", '6.0',
            "-Y", '0.0'
        ],
        output="screen",
    )


    return LaunchDescription(
		[
			ign_sim,
			ign_spawn_entity,
		]
	)
