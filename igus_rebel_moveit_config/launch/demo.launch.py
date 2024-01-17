
from launch.substitutions import PathJoinSubstitution
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
    
    gripper_arg = DeclareLaunchArgument(
        name="gripper",
        default_value="none",
        choices=["none", "camera"],
        description="Gripper mount to attach to the last joint",
    )

    camera_type_arg = DeclareLaunchArgument(
        name="camera_type",
        default_value="realsense",
        choices=["realsense", "oakd"],
        description="Which camera to attach to the mount",
    )

    hardware_protocol_arg = DeclareLaunchArgument(
        name="hardware_protocol",
        default_value="simulation",
        choices=["mock_hardware", "cri", "simulation"],
        description="Which hardware protocol or mock hardware should be used",
    )

    rviz_file_arg = DeclareLaunchArgument(
        name="rviz_file",
        default_value="none",
        description="Path to the RViz configuration file",
    )

    load_octomap_arg = DeclareLaunchArgument(
        name="load_octomap",
        default_value="false",
        description="Load the octomap server inside the planning scene",
        choices=["true", "false"],
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Argument for choose to use simulation time or not",
    )

    return LaunchDescription(
        [
            load_base_arg,
            gripper_arg,
            camera_type_arg,
            hardware_protocol_arg,
            rviz_file_arg,
            load_octomap_arg,
            use_sim_time_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs):
    

    # Paths
    moveit_launch_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_moveit_config"),
            "launch",
            "moveit_controller.launch.py",
        ]
    )


    # Ignition launch file
    ignition_launch_file = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("igus_rebel_gazebo_ignition"), '/launch', '/ignition.launch.py'])
        )



    # Moveit launch file
    igus_rebel_moveit_config_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
    )


    return [
            igus_rebel_moveit_config_launch,
            ignition_launch_file,
        ]
