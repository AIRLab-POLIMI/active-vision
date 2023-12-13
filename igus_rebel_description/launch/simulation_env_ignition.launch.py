import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions
from launch.actions import ExecuteProcess, Shutdown, TimerAction
from ament_index_python.packages import get_package_share_directory

 
 
def generate_launch_description():
 
    # Constants for paths to different files and folders
    package_name = 'igus_rebel_description'
    pkg_path = FindPackageShare(package=package_name).find(package_name)

    gazebo_models_path = 'models'
    gazebo_models_path = os.path.join(pkg_path, gazebo_models_path)
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = gazebo_models_path

    rviz_config_file_path = 'rviz/config.rviz'
    default_rviz_config_path = os.path.join(pkg_path, rviz_config_file_path)

    urdf_mod_file_path = 'urdf/igus_rebel.urdf.xacro'
    urdf_upg_file_path = 'urdf/igus_rebel_ros2.urdf.xacro'
    default_urdf_mod_model_path = os.path.join(pkg_path, urdf_mod_file_path)
    default_urdf_upg_model_path = os.path.join(pkg_path, urdf_upg_file_path)

    world_file_path = 'worlds/tomato_field.sdf'
    world_path = os.path.join(pkg_path, world_file_path)

        
    # Pose where we want to spawn the robot
    spawn_x_val = '-2.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.00'

    
    
    # Launch configuration variables specific to simulation
    urdf = LaunchConfiguration('urdf')
    gripper = LaunchConfiguration('gripper')
    hardware_protocol = LaunchConfiguration('hardware_protocol')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    namespace = LaunchConfiguration('namespace')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')
    use_namespace = LaunchConfiguration('use_namespace')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
    
    # Declare the launch arguments 
    declare_urdf_type = DeclareLaunchArgument(
        name="urdf",
        default_value="mod",
        description="Robot Description file to use",
        choices=["mod", "upg"],
    )

    declare_gripper = DeclareLaunchArgument(
        name="gripper",
        default_value="none",
        choices=["none", "camera"],
        description="Which gripper mount to attach to the flange",
    )

    declare_hardware = DeclareLaunchArgument(
		name="hardware_protocol",
		default_value="simulation",
		choices=["mock_hardware", "cri", "simulation"],
		description="Which hardware protocol or mock hardware should be used",
	)

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='False',
        description='Flag to enable joint_state_publisher_gui')
        
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the navigation stack')
                
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')
    
    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')
    
    declare_urdf_mod_model_path_cmd = DeclareLaunchArgument(
        name='urdf_mod_model', 
        default_value=default_urdf_mod_model_path, 
        description='Absolute path to robot urdf file')
    
    declare_urdf_upg_model_path_cmd = DeclareLaunchArgument(
        name='urdf_upg_model', 
        default_value=default_urdf_upg_model_path, 
        description='Absolute path to robot urdf file')
        
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')
    
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')
    
    # NODES
    
    # A GUI to manipulate the joint state values
    start_joint_state_publisher_gui_node = Node(
        condition=IfCondition(gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui')
    
    
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_mod_cmd = Node(
        condition=LaunchConfigurationEquals('urdf', 'mod'),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        parameters=[{'use_sim_time': use_sim_time, 
        'robot_description': launch_ros.descriptions.ParameterValue(Command([ FindExecutable(name="xacro"), " ", default_urdf_mod_model_path, " gripper:=" ,LaunchConfiguration("gripper"), " hardware_protocol:=", LaunchConfiguration("hardware_protocol"),]))}])

    start_robot_state_publisher_upg_cmd = Node(
        condition=LaunchConfigurationEquals('urdf', 'upg'),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        parameters=[{'use_sim_time': use_sim_time}, 
        {'robot_description': launch_ros.descriptions.ParameterValue(Command([ FindExecutable(name="xacro"), " ", default_urdf_upg_model_path, " gripper:=" ,LaunchConfiguration("gripper"), " hardware_protocol:=", LaunchConfiguration("hardware_protocol"),]))}])
    
    # Launch RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )
    

    # Gazebo GUI configuration file
    gazebo_config_gui_file = os.path.join(
		get_package_share_directory("igus_rebel_description"),
		"config",
		"gazebo_gui.config",
	)
     

    gz_sim = ExecuteProcess(
		cmd=[
			"ign gazebo",
			"--verbose 1 -r --gui-config " + gazebo_config_gui_file,
			world_path,
		],
		output="log",
		shell=True,
	)

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "igus_rebel",
            "-z", spawn_z_val,
            "-x", spawn_x_val,
            "-y", spawn_y_val,
            "-Y", spawn_yaw_val
        ],
        output="screen",
    )

    # parameter for controller
    joint_names_list=["joint1","joint2","joint3",
                    "joint4","joint5","joint6"]
    ign_joint_topics_list=[]
    for joint_name in joint_names_list:
        ign_joint_topics_list.append("/model/igus_rebel/joint/%s/0/cmd_pos"%joint_name)
    
    joint_state_publisher = Node(
        package='igus_rebel_description',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{"joint_names": joint_names_list},
                {"ign_topic": "/world/tomato_field/model/igus_rebel/joint_state"},
            ],
        output='screen')
    
    joint_controller=Node(
        package='igus_rebel_description', 
        executable='joint_controller',
        name="joint_controller",
        parameters=[{"joint_names": joint_names_list},
            {"ign_joint_topics": ign_joint_topics_list},
            {"rate":200},
        ],
        output='screen') 
    

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            'config_file': os.path.join(pkg_path, 'config', 'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output="screen",
    )

    depth_stf = Node(package='tf2_ros', executable='static_transform_publisher',
            namespace = 'igus_rebel_description',
            name = 'depth_stf',
                arguments = [
                    '0', '0', '0', '-1.5707963267948966', '0', '-1.5707963267948966',
                    'oakd_link',
                    'igus_rebel/link_8/depth_camera'
            ])
    
    point_stf = Node(package='tf2_ros', executable='static_transform_publisher',
            namespace = 'igus_rebel_description',
            name = 'depth_stf',
                arguments = [
                    '0', '0', '0', '0', '0', '0',
                    'oakd_link',
                    'igus_rebel/link_8/pointcloud'
            ])
    
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_urdf_type)
    ld.add_action(declare_gripper)
    ld.add_action(declare_hardware)
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_urdf_mod_model_path_cmd)
    ld.add_action(declare_urdf_upg_model_path_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)  
    ld.add_action(declare_use_rviz_cmd) 
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)
    

    ld.add_action(gz_sim)
    ld.add_action(gz_spawn_entity)
    #ld.add_action(joint_state_publisher)
    ld.add_action(joint_controller)  
    ld.add_action(gz_ros2_bridge)

    ld.add_action(start_robot_state_publisher_mod_cmd)
    ld.add_action(start_robot_state_publisher_upg_cmd)
    #ld.add_action(start_joint_state_publisher_gui_node)
    ld.add_action(start_rviz_cmd)
    ld.add_action(depth_stf)
    ld.add_action(point_stf)

 

    return ld