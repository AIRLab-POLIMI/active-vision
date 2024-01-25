# Based on the visualize_franka script licensed under the Apache license which can be found here:
# https://github.com/frankaemika/franka_ros2/blob/develop/franka_description/launch/visualize_franka.launch.py

import os
from os import environ
from launch import LaunchDescription

from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    return LaunchDescription(
        [
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

    gazebo_config_gui_path = os.path.join(
        get_package_share_directory("igus_rebel_gazebo_ignition"),
        "config",
        "gazebo_gui.config",
    )



    # Additional bridge for joint state if Moveit is not used (only for visualization of the description)
    if LaunchConfiguration("moveit").perform(context) == 'false':
        bridge_config_path = os.path.join(
            get_package_share_directory("igus_rebel_gazebo_ignition"), 
            "config", 
            "bridge_description.yaml",
        )
    else:
        bridge_config_path = os.path.join(
            get_package_share_directory("igus_rebel_gazebo_ignition"), 
            "config", 
            "bridge_moveit.yaml",
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
            "-z", '0.0',
            "-x", '-2.0',
            "-y", '0.0',
            "-Y", '-1.0'
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



    # Static TFs to solve depth sensor bug (only used when virtual camera is used on Ignition)
    if LaunchConfiguration("camera").perform(context) != 'none' and LaunchConfiguration("gripper").perform(context) != 'none':

        depth_stf = Node(package='tf2_ros', executable='static_transform_publisher',
            namespace = 'igus_rebel_gazebo_ignition',
            name = 'depth_stf',
            arguments = [
                '0', '0', '0', '-1.5707963267948966', '0', '-1.5707963267948966',
                'camera_link',
                'igus_rebel/link_8/depth_camera'
            ],
            parameters=[{'use_sim_time': use_sim_time},],
        )
        
        point_stf = Node(package='tf2_ros', executable='static_transform_publisher',
            namespace = 'igus_rebel_gazebo_ignition',
            name = 'point_stf',
                arguments = [
                    '0', '0', '0', '0', '0', '0',
                    'camera_link',
                    'igus_rebel/link_8/pointcloud'
            ],
            parameters=[{'use_sim_time': use_sim_time},],
        )

        return_actions.append(depth_stf)
        return_actions.append(point_stf)




    # Returns
    return_actions.append(ign_sim)
    return_actions.append(ign_spawn_entity)
    return_actions.append(ign_bridge)
    


    return return_actions

