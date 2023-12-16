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

    # Paths
    world_path = os.path.join(
        get_package_share_directory("igus_rebel_description_ros2"), 
        "worlds", 
        "tomato_field.sdf",
    )

    gazebo_config_gui_path = os.path.join(
        get_package_share_directory("igus_rebel_description_ros2"),
        "config",
        "gazebo_gui.config",
    )

    bridge_config_path = os.path.join(
        get_package_share_directory("igus_rebel_moveit_config"), 
        "config", 
        "bridge.yaml",
    )

    # Processes
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

    # parameter for controller
    joint_names_list=["joint1","joint2","joint3",
                    "joint4","joint5","joint6"]
    ign_joint_topics_list=[]
    for joint_name in joint_names_list:
        ign_joint_topics_list.append("/model/igus_rebel/joint/%s/0/cmd_pos"%joint_name)
    
    joint_controller=Node(
        package='igus_rebel_moveit_config', 
        executable='joint_controller',
        name="joint_controller",
        parameters=[{"joint_names": joint_names_list},
            {"ign_joint_topics": ign_joint_topics_list},
            {"rate":200},
        ],
        output='screen') 
    

    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            'config_file': bridge_config_path,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output="screen",
    )


    return LaunchDescription(
		[
			ign_sim,
			ign_spawn_entity,
            joint_controller,
		]
	)
