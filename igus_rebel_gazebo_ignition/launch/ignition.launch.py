# Based on the visualize_franka script licensed under the Apache license which can be found here:
# https://github.com/frankaemika/franka_ros2/blob/develop/franka_description/launch/visualize_franka.launch.py

import os
from os import environ
from launch import LaunchDescription

from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    spawn_x_arg = DeclareLaunchArgument(
        name="spawn_x",
        default_value="-2.0",
        description="x position for the robot spawned in Gazebo Ignition",
    )

    spawn_y_arg = DeclareLaunchArgument(
        name="spawn_y",
        default_value="0.0",
        description="y position for the robot spawned in Gazebo Ignition",
    )

    spawn_z_arg = DeclareLaunchArgument(
        name="spawn_z",
        default_value="0.0",
        description="z position for the robot spawned in Gazebo Ignition",
    )

    spawn_yaw_arg = DeclareLaunchArgument(
        name="spawn_yaw",
        default_value="-1.0",
        description="Y position for the robot spawned in Gazebo Ignition",
    )

    return LaunchDescription(
        [
            spawn_x_arg,
            spawn_y_arg,
            spawn_z_arg,
            spawn_yaw_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs):


    # Array of action that will be returned at the end for execution
    return_actions = []
     
     

    # Sim time
    if LaunchConfiguration("use_sim_time").perform(context) == 'true':
        use_sim_time = True
    else:
        use_sim_time = False



    # Ignition env variables
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



    # Paths
    world_path = os.path.join(
        get_package_share_directory("igus_rebel_description_ros2"), 
        "worlds", 
        "tomato_field.sdf",
    )


    # Additional bridge for joint state if Moveit is not used (only for visualization of the description)
    # and related gui with or without the joint position controller gui
    if LaunchConfiguration("moveit").perform(context) == 'false':
        bridge_config_path = os.path.join(
            get_package_share_directory("igus_rebel_gazebo_ignition"), 
            "config", 
            "bridge_description.yaml",
        )

        gazebo_config_gui_path = os.path.join(
            get_package_share_directory("igus_rebel_gazebo_ignition"),
            "config",
            "gazebo_gui_description.config",
        )

    else:
        bridge_config_path = os.path.join(
            get_package_share_directory("igus_rebel_gazebo_ignition"), 
            "config", 
            "bridge_moveit.yaml",
        )

        gazebo_config_gui_path = os.path.join(
            get_package_share_directory("igus_rebel_gazebo_ignition"),
            "config",
            "gazebo_gui_moveit.config",
        )




    # Ignition processes
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
            "-x", LaunchConfiguration("spawn_x"),
            "-y", LaunchConfiguration("spawn_y"),
            "-z", LaunchConfiguration("spawn_z"),
            "-Y", LaunchConfiguration("spawn_yaw")
        ],
        parameters=[{'use_sim_time': use_sim_time},],
        output="screen",
    )

    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'use_sim_time': use_sim_time},
            {'config_file': bridge_config_path,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output="screen",
    )


    # Returns
    return_actions.append(ign_sim)
    return_actions.append(ign_spawn_entity)
    return_actions.append(ign_bridge)
    


    return return_actions

