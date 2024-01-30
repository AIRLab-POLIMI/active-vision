
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    load_base_arg = DeclareLaunchArgument(
        name="load_base",
        default_value="false",
        description="Load the mobile robot model and tower",
        choices=["true", "false"],
    )
    
    mount_arg = DeclareLaunchArgument(
        name="mount",
        default_value="none",
        choices=["none", "mount_v1"],
        description="Mount to attach to the last joint",
    )

    camera_arg = DeclareLaunchArgument(
        name="camera",
        default_value="none",
        choices=["realsense", "oakd", "none"],
        description="Which camera to attach to the mount",
    )

    end_effector_arg = DeclareLaunchArgument(
        name="end_effector",
        default_value="none",
        choices=["toucher_cylinder", "none"],
        description="Which end_effector to attach to the mount",
    )

    hardware_protocol_arg = DeclareLaunchArgument(
        name="hardware_protocol",
        default_value="simulation",
        choices=["mock_hardware", "cri", "simulation", "ignition"],
        description="Which hardware protocol or mock hardware should be used",
    )

    load_gazebo_arg = DeclareLaunchArgument(
        name="load_gazebo",
        default_value="false",
        choices=["true", "false"],
        description="Which Gazebo version to launch",
    )

    load_octomap_arg = DeclareLaunchArgument(
        name="load_octomap",
        default_value="false",
        description="Load the octomap server inside the planning scene",
        choices=["true", "false"],
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Argument for choose to use simulation time or not",
    )

    moveit_arg = DeclareLaunchArgument(
        name="moveit",
        default_value="true",
        choices=["true", "false"],
        description="Argument for tell ROS that we are using moveit or not",
    )

    return LaunchDescription(
        [
            load_base_arg,
            mount_arg,
            camera_arg,
            end_effector_arg,
            hardware_protocol_arg,
            load_gazebo_arg,
            load_octomap_arg,
            use_sim_time_arg,
            moveit_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs):
    

    # Array of action that will be returned at the end for execution
    return_actions = []



    # Ignition launch file
    if LaunchConfiguration("load_gazebo").perform(context) == "true":

        ignition_launch_file = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("igus_rebel_gazebo_ignition"), '/launch', '/ignition.launch.py']
            ),
        )

        return_actions.append(ignition_launch_file)


    # Moveit launch file
    moveit_config_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("igus_rebel_moveit_config"), '/launch', '/moveit_controller.launch.py']
        ),
    )



    # Returns
    return_actions.append(moveit_config_launch_file)

    return return_actions
