
<h1 align="center">
   Active Vision and Zero-Shot Learning for Enhancing Agricultural Environment Perception
</h1>

<p align="center">
  Implementation of the research project developed during the Master's thesis of Michele Carlo La Greca at Politecnico di Milano, corresponding to the work related to the <a href="https://arxiv.org/abs/2409.12602">preprint</a> available on <i>arXiv</i>.
  <p align="center">
    <img src="images/main.png" alt="Main Image" width="80%">
  </p>
</p>

<hr>


[![humble][humble-badge]][humble]
[![ubuntu22][ubuntu22-badge]][ubuntu22]

<!-- ![GitHubWorkflowStatus](https://img.shields.io/github/actions/workflow/status/AIRLab-POLIMI/active-vision/main.yml?logo=github&style=flat-square)
[![GitHubcontributors](https://img.shields.io/github/contributors/AIRLab-POLIMI/active-vision?style=flat-square)](CONTRIBUTING.md)
[![License](https://img.shields.io/github/license/AIRLab-POLIMI/active-vision?style=flat-square)](LICENSE) -->

<hr>

## Table of contents
  * [Introduction](#introduction)
  * [Installation](#installation)
  * [Visualization](#visualization)
  * [OctoMap Creation](#octomap-creation)
  * [Active Vision](#active-vision) 
  * [Contributing](CONTRIBUTING.md)
  * [License](LICENSE)

<hr>

# Introduction

Agriculture is essential to society. Robotics can boost productivity in agriculture, and robots must be able to accurately perceive the unstructured, dynamic, and possibly covered environment of plants and crops. This complexity presents significant challenges for traditional management methods. The proposed research introduces an approach to overcome the challenges and complexities of fruit perception through Active Vision (AV). Instead of relying on passive observation, AV allows robots to actively perceive, explore, and reconstruct at run-time their surroundings by planning the optimal position of the camera viewpoint using the Next-Best View (NBV) planning, which maximizes the information gained regarding plants and crops. This ensures that even hidden or occluded parts of the environment are effectively captured.

This work applies Zero-Shot Learning (ZSL) to provide useful segmentation, enabling the robot to generalize and adapt to various crops or environmental features without requiring specific training data for each scenario. By leveraging both 3D and semantic data, the robot can reconstruct a detailed, semantic, and context-aware map of the environment, allowing it to strategically adjust its movements and positioning, leading to more effective interactions with the environment.

This work focuses on the following contributions:  
1. Developed a modular architecture in ROS 2, C++, and Python for Active Vision in agricultural robotics, addressing the challenge of detecting occluded fruits.  
2. To the best of the author’s knowledge, this is the first work to integrate Zero-Shot Learning with Active Vision exploration, enabling environment-independent operation in agriculture.  
3. Conducted extensive evaluations both in simulation and real-world scenarios, in contrast to state-of-the-art methods that primarily focus on simulated environments with supervised learning.  
4. Set a benchmark standard for the lack of reproducibility and availability of open-source code in the context of Active Vision in agricultural robotics.

    

# Installation
  
<details>
  <summary>
    Step 1: Install the ROS 2 Humble distribution for Ubuntu 22.04 and other useful elements. 
  </summary>

   - Guide at [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
     - Regarding the sourcing, instead of always running the command `source /opt/ros/humble/setup.bash`, a good practice is to add this line in the end of the *bashrc* file that can be modified with the command `gedit ~/.bashrc`. After the modification, the file needs to be saved and the terminal restarted.
   - Install RQt tools: `sudo apt install ros-humble-rqt*`.
   - Install dev tools: `sudo apt update && sudo apt install -y python3-vcstool`.
   - Install Gazebo Ignition Fortress:
     - A full guide is at [Gazebo Ignition Fortress](https://gazebosim.org/docs/fortress/install_ubuntu).
     - After the installation run the command: `export IGNITION_VERSION=fortress`.
     - Install ROS 2 - Gazebo Ignition integration tools from [this repository](https://github.com/gazebosim/ros_gz/tree/humble):
       - ```bash
         sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
         ```
       - ```bash
         curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null
         ```
       - ```bash
         sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
         ```
       - ```bash
         sudo apt-get update
         ```
       - ```bash
         sudo apt install ros-humble-ros-gz
         ```
       
</details>


<details>
  <summary>
    Step 2: Install the dependencies and requirements of the Igus ReBeL ROS 2 repository.
  </summary>

   - Install MoveIt2:
     - Guide at [MoveIt2](https://moveit.ros.org/install-moveit2/source/)
     - Create a workspace (for example called `moveit2_ws`) and create a folder `src` inside
     - Add in the *bashrc* file: `source ../moveit2_ws/install/setup.bash`
     - In `movit2_ws/src` folder clone the repository: `git clone https://github.com/ros-planning/moveit2.git -b $ROS_DISTRO`
     - In the same folder: `for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y`
     - In `movit2_ws` folder: `MAKEFLAGS="-j4" colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release`
     - Install STOMP planner and, since it is not supported for the humble version of ROS 2, some manual modifications are needed:
       - Inside `moveit2_ws/src/moveit2/moveit_configs_utils/deafult_configs` copy the file *stomp_planning.yaml* (available on the [MoveIt2’s github page](https://github.com/moveit/moveit2) in the same folder)
       - Inside `moveit2_ws/src/moveit2/moveit_planners/moveit_planners/package.xml` insert: `<exec_depend>moveit_planners_stomp</exec_depend>`
       - Download the *Stomp* folder of the [MoveIt2’s github page](https://github.com/moveit/moveit2) (inside moveit2/moveit_planners) and place it in the same local folder (or follow the [instruction](https://github.com/moveit/stomp_moveit) to build it from source)
       - Go in folder `moveit2_ws/src/moveit2/moveit_planners` and run: `rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO`
       - In `moveit2_ws` folder: `colcon build --packages-select moveit_planners_stomp`
       - If an error raises with respect to the `#include <moveit/utils/logger.hpp>` not found:
         - go in the repository `moveit2/moveit_core/utils` and from the include and the src folder copy in the local repo the files regarding it: `logger.hpp` and `logger.cpp`
         - in folder `moveit2_ws`: `colcon build --packages-select moveit_core`
       - To solve other errors when the stomp package is compiled:
         - In `moveit2_ws/src/moveit2/moveit_planners/stomp/include/stomp_moveit/stomp_moveit_planning_context.hpp` replace the void type of the two solve functions (line 59 and 61) in bool, as well as in the corresponding source file (lines 280 and 215). Moreover, in the src file change the returns of these modified 2 function from empty to false and return true at the end of the function at line 279.
         - In `moveit2_ws/src/moveit2/moveit_planners/stomp/src/stomp_moveit_planning_context.cpp` comment line 220 removing *planning_id*, change all `res.error_code` in `res.error_code_`, all `res.planning_time` in `res.planning_time_`, and all `res.trajectory` in `res.trajectory_`
       - In `moveit2_ws` folder: `colcon build --packages-select moveit_planners_stomp`
     - Install `mobile_manipulation_interfaces` package created by Simone Giampà in his [work](https://github.com/AIRLab-POLIMI/mobile-manipulation-scout-rebel):
       - Download and put the package into the `moveit_ws/src`, then build only this: `colcon build --packages-select mobile_manipulation_interfaces`
     - Install `moveit2_api` package created by Simone Giampà in his [work](https://github.com/AIRLab-POLIMI/mobile-manipulation-scout-rebel): 
       <details>
         <summary>
           OPTION 1: Use the active-vision-config branch:
         </summary>
          
       - Download from the active-vision-branch the specific package folder `moveit2_api` and put it into the `moveit_ws/src` folder
       - In the folder `moveit2_ws`: `colcon build --packages-select moveit2_api`
         
       </details>
       <details>
         <summary>
           OPTION 2: Use the main branch:
         </summary>
          
       - Download from the main branch the specific package folder and put it into the `moveit_ws/src`
       - Into the `moveit2_api.hpp` file:
         - Change `visual_tools` variable from private to public
         - Remove `const` from the declaration of variable `camera_frame_name` (line 57)
         - in the 3 functions `robotPlanAndMove()`, add a string argument to define the name of the target pose. Same in the `moveit2_api.cpp` file:
           - ```bash
               bool robotPlanAndMove(geometry_msgs::msg::PoseStamped::SharedPtr target_pose, std::string target, bool compensation=true);
               ...
            	double robotPlanAndMove(std::vector<geometry_msgs::msg::Pose> pose_waypoints, std::string target);
               ...
            	bool robotPlanAndMove(std::array<double, 6> joint_space_goal, std::string target);
             ``` 
       - Into the `moveit2_api.cpp` file:
         - Remove the publish text from the init method of the visual tools
         - ```bash
           # from
            camera_frame_name(get_parameter("camera_frame").as_string()) { //line 11
            ...
            load_base_arg = this->get_parameter("load_base").as_bool(); //line 13
            ...
            bool loaded = moveit2_node_->get_parameter("planning_plugin", planner_plugin_name); //line 133
            
            
            # to
            camera_frame_name("camera_frame") { // line 11
            
            # load base = true if the robotic arm is mounted on the mobile robot base, false if it's mounted on a table
            camera_frame_name = this->declare_parameter("camera_frame", camera_frame_name);
            load_base_arg = this->declare_parameter<bool>("load_base", load_base_arg);

            ...
            planner_plugin_name = moveit2_node_->declare_parameter("planning_plugin", planner_plugin_name);
            bool loaded = moveit2_node_->get_parameter("planning_plugin", planner_plugin_name); 
           ```
         - Create a function that convert a goal in the joint space into a goal in the cartesian space:
           ```bash
           Eigen::Isometry3d MoveIt2APIs::fromJointSpaceGoalToCartesianPose(std::array<double, 6> joint_space_goal) {
            	moveit::core::RobotState goal_state(*move_group->getCurrentState());
            	std::vector<double> joint_space_goal_vector(joint_space_goal.begin(), joint_space_goal.end());
            	goal_state.setJointGroupPositions(joint_model_group, joint_space_goal_vector);
            
            	move_group->setStartState(*move_group->getCurrentState());
            	move_group->setGoalPositionTolerance(position_tolerance);		// meters ~ 5 mm
            	move_group->setGoalOrientationTolerance(orientation_tolerance); // radians ~ 5 degrees
            	move_group->setPlanningTime(timeout_seconds);
            	move_group->setPlanningPipelineId("stomp");
            	move_group->setPlannerId("STOMP");
            	move_group->setMaxVelocityScalingFactor(max_velocity_scaling_joint_space);
            	move_group->setMaxAccelerationScalingFactor(max_acceleration_scaling);
            
            	bool valid_motion = move_group->setJointValueTarget(goal_state);
            	if (!valid_motion) {
            		throw std::runtime_error("Target joints outside their physical limits");
            	}
            
            	const Eigen::Isometry3d goal_pose = goal_state.getGlobalLinkTransform(end_effector_link);
            	Eigen::Isometry3d goal_pose_tf2;
            
            	geometry_msgs::msg::TransformStamped tf_base_footprint_msg;
            	try {
            		// lookup transform from root base frame (base_footprint when load_base = true) to fixed base frame (igus rebel base link)
            		tf_base_footprint_msg = tf_buffer_->lookupTransform(fixed_base_frame, root_base_frame, tf2::TimePointZero);
            	} catch (const tf2::TransformException &ex) {
            		RCLCPP_ERROR(LOGGER, "%s", ex.what());
            		throw std::runtime_error(ex.what());
            	}
            
            	tf2::doTransform(goal_pose, goal_pose_tf2, tf_base_footprint_msg); // in, out, transform
            
            	return goal_pose_tf2;
            }
           ```
         - Add a bool compensation to the robotPlanAndMove function, that by default is true
           ```bash
            # Definition
            bool robotPlanAndMove(geometry_msgs::msg::PoseStamped::SharedPtr target_pose, std::string target, bool compensation=true);
            
            # Implementation
            bool MoveIt2APIs::robotPlanAndMove(geometry_msgs::msg::PoseStamped::SharedPtr target_pose, std::string target, bool compensation) {
            	RCLCPP_INFO(LOGGER, "Planning and moving to %s pose", target.c_str());
            
            	geometry_msgs::msg::PoseStamped::UniquePtr compensated_target_pose;
            	if (compensation){
            		// apply offset compensation to the cartesian space target pose
            		compensated_target_pose = compensateTargetPose(*target_pose);
            	}
            	else{
            		compensated_target_pose = std::make_unique<geometry_msgs::msg::PoseStamped>(*target_pose);
            	}
            
            	if (!checkIKSolution(compensated_target_pose->pose)) {
            		RCLCPP_ERROR(LOGGER, "No valid IK solution for the %s pose", target.c_str());
            		//return false;
            	}
            	...
           ```
         - Add to each of the 3 robotPlanAndMove function at the beginning:
           ```bash
           RCLCPP_INFO(LOGGER, "Planning and moving to %s pose", target.c_str());
           ```
         - Change `trajectory` to `trajectory_` in line, 539, 540, 717.
       - Go in `moveit2_ws` and build: `colcon build --packages-select moveit2_api`
     

       
   - Install MoveIt Visual Tools:
     - Go in `moveit2_ws` folder and clone the repository: `git clone -b ros2 https://github.com/ros-planning/moveit_visual_tools`
     - In the same folder: `vcs import < moveit_visual_tools/moveit_visual_tools.repos`
     - In the same folder: `rosdep install -r --from-paths . --ignore-src --rosdistro humble -y`
     - In the folder `movit2_ws`:
       - `colcon build --packages-select rviz_visual_tools`
       - `colcon build --packages-select graph_msgs`
       - `colcon build --packages-select moveit_visual_tools`
         -  If an error raises with respect to *trajectory* variable, change it manually in the source code.
   
   
   - Install the ROS 2 wrapper of the Realsense D435 used in the project:
     - Guide at [this repository](https://github.com/IntelRealSense/realsense-ros)
     - First, install the latest Intel® RealSense™ SDK 2.0. Use the Option 2, consisting of installing librealsense2:
       - `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list`
       - `sudo apt install curl # if you haven't already installed curl`
       - `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
       - `sudo apt install ros-humble-librealsense2*`
     - Install the ROS 2 wrapper:
       - `sudo apt install ros-humble-realsense2-*`

         
   - Install the hardware interface for controlling the Gazebo Ignition robot through MoveIt2:
     - Create a workspace (for example called `gz_ros2_control`) and create a folder `src` inside
     - Add in the *bashrc* file: `source ../gz_ros2_control/install/setup.bash`
     - In `gz_ros2_control/src` folder clone the humble branch of the repository containing the hardware interface for Gazebo Ignition: `git clone --single-branch --branch humble https://github.com/ros-controls/gz_ros2_control`
     - In the same folder: `rosdep install -r --from-paths . --ignore-src --rosdistro humble -y`
     - In `gz_ros2_control` folder: `colcon build`

   - Install ros2_control: `sudo apt install ros-$ROS_DISTRO-ros2-control`
   - Install ros2_controllers: `sudo apt install ros-$ROS_DISTRO-ros2-controllers`
   - Install tf_transformations: `sudo apt install ros-$ROS_DISTRO-tf-transformations`
  
   - Install AgileX Scout description package:
     - Download the main branch and place only the folder `agilex_scout` into a new workspace `agilex_scout/src`
     - In the file `agilex_scout/src/agilex_scout/urdf/mobile_robot/scout_v2.urdf.xacro` comment or remove the 82-94 lines, so that wheels are not loaded (to solve an error).
     - In the workspace folder `agilex_scout`: `colcon build`
     - Add in the *bashrc* file: `source ../agilex_scout/install/setup.bash`
  
  
</details>


<details>
  <summary>
    Step 3: Clone the active vision branch of the <a href="https://github.com/AIRLab-POLIMI/ros2-igus-rebel">Igus ReBeL ROS 2</a> repository.
  </summary>
   
   - While the master branch is the general integration of the Igus ReBeL robotic arm into ROS 2, the active vision branch has some elements strictly related to the current project and some packages have been removed (servo, commander and gripper controller).
   - Create a workspace (for example called `ros2_igus_rebel`) and create a folder `src` inside
   - Add in the *bashrc* file: `source ../ros2_igus_rebel/install/setup.bash`
   - In `ros2_igus_rebel/src` folder run: `git clone --single-branch --branch active-vision-config https://github.com/AIRLab-POLIMI/ros2-igus-rebel.git`
   - `sudo rosdep init`
   - `rosdep update`
   - `rosdep install -i --from-path src --rosdistro humble -y` to install the dependencies
   - `pip install xacro`
   - In the workspace folder run: `colcon build`
   - `source ~/.bashrc` to make effective the changes (or restart the terminal)
       
</details>


<details>
  <summary>
    Step 4: Install the dependencies of this repository.
  </summary>
   
   - Install *depth_image_proc* package:
      ```bash
       sudo apt install ros-humble-depth-image-proc*
       ```
   - Install *PCL*:
      ```bash
      sudo apt-get install ros-humble-pcl-ros
      ```
   - Install *Octomap* library:
     - From [this repostory](https://github.com/octomap/octomap/releases) download the latest version as source file (.zip)
     - Create a workspace `octomap_ws/src` where the content of the .zip file is extracted
     - In folder `octomap_ws`: `colcon build`
     - Source: `source ../octomap_ws/install/setup.bash`
     - Ignore the warnings of the deprecated functions used in the package
     - Re-build with `colcon build --cmake-arg -DCMAKE_BUILD_TYPE=Release` if it is needed to have the flag release (suggested)
       <details>
         <summary>
           a warning may raise:
         </summary>
       
            ```bash
            CMake Warning at CMakeLists.txt:39 (add_library):
            Cannot generate a safe runtime search path for target moveit_visual_tools
            because there is a cycle in the constraint graph:
            ...
            
            dir 0 is [/home/michelelagreca/Documents/robotics/movit2_ws/install/graph_msgs/lib]
            dir 1 is [/home/michelelagreca/Documents/robotics/movit2_ws/install/moveit_ros_planning/lib]
            dir 2 is [/home/michelelagreca/Documents/robotics/movit2_ws/install/rviz_visual_tools/lib]
            dir 3 is [/home/michelelagreca/Documents/robotics/movit2_ws/install/moveit_ros_occupancy_map_monitor/lib]
            dir 4 is [/home/michelelagreca/Documents/robotics/movit2_ws/install/moveit_core/lib]
            dir 5 is [/opt/ros/humble/lib]
              dir 7 must precede it due to runtime library [libimage_transport.so]
            dir 6 is [/home/michelelagreca/Documents/robotics/octomap_ws/install/octomap-distribution/lib]
              dir 7 must precede it due to runtime library [liboctomap.so.1.9]
            dir 7 is [/opt/ros/humble/lib/x86_64-linux-gnu]
              dir 6 must precede it due to runtime library [liboctomap.so.1.9]
            dir 8 is [/home/michelelagreca/Documents/robotics/movit2_ws/install/moveit_msgs/lib]
            dir 9 is [/opt/ros/humble/opt/rviz_ogre_vendor/lib]
            ...
                        
            Some of these libraries may not be found correctly.
            
            CMake Warning at CMakeLists.txt:52 (add_executable):
            Cannot generate a safe runtime search path for target
            moveit_visual_tools_demo because there is a cycle in the constraint graph:
            ...
            
            dir 0 is [/home/michelelagreca/Documents/robotics/movit2_ws/build/moveit_visual_tools]
            dir 1 is [/home/michelelagreca/Documents/robotics/movit2_ws/install/graph_msgs/lib]
            dir 2 is [/home/michelelagreca/Documents/robotics/movit2_ws/install/moveit_ros_planning/lib]
            dir 3 is [/home/michelelagreca/Documents/robotics/movit2_ws/install/rviz_visual_tools/lib]
            dir 4 is [/home/michelelagreca/Documents/robotics/movit2_ws/install/moveit_ros_occupancy_map_monitor/lib]
            dir 5 is [/home/michelelagreca/Documents/robotics/movit2_ws/install/moveit_core/lib]
            dir 6 is [/opt/ros/humble/lib]
              dir 8 must precede it due to runtime library [libimage_transport.so]
            dir 7 is [/home/michelelagreca/Documents/robotics/octomap_ws/install/octomap-distribution/lib]
              dir 8 must precede it due to runtime library [liboctomap.so.1.9]
            dir 8 is [/opt/ros/humble/lib/x86_64-linux-gnu]
              dir 7 must precede it due to runtime library [liboctomap.so.1.9]
            dir 9 is [/home/michelelagreca/Documents/robotics/movit2_ws/install/moveit_msgs/lib]
            dir 10 is [/opt/ros/humble/opt/rviz_ogre_vendor/lib]
            ...
            
            Some of these libraries may not be found correctly. 
            ```
        </details>
   
   - Install `Octomap_Server2`:
     - Create a workspace `octomap_server2_ws/src`
     - In the folder `octomap_server2_ws/src`: `git clone https://github.com/iKrishneel/octomap_server2.git`
     - In the folder `octomap_server2_ws/src`: `git clone --single-branch --branch ros2 https://github.com/OctoMap/octomap_msgs.git`
     - Install dependencies: `rosdep install -r --from-paths . --ignore-src --rosdistro humble -y`
     - In folder `octomap_server2_ws`: `sudo apt-get purge --auto-remove ros-humble-octomap-msgs`
     - In folder `octomap_server2_ws`: `colcon build --symlink-install --packages-select octomap_msgs`
     - In folder `octomap_server2_ws`: `colcon build --symlink-install --packages-select octomap_server2`
     - Ignore warnings
     - Source: `source ../octomap_server2_ws/install/setup.bash`

   - Install *Lang SAM*:
     - ```bash
       pip install torch torchvision
       pip install -U git+https://github.com/luca-medeiros/lang-segment-anything.git
       ```
     - Add `/home/.../.local/bin` to the PATH
       - Open a terminal and edit your shell configuration file (e.g., .bashrc for Bash users or .zshrc for Zsh users) with a text editor:
         ```bash
         gedit ~/.bashrc
       - Add the following line at the end of the file:
         ```bash
         export PATH="$PATH:/home/michelelagreca/.local/bin"
         ```
       - ```bash
         source ~/.bashrc
         ```
    
   - Install *YOLO World + EfficentViT SAM*:
     - Move inside the `../.local/lib/pythonX.XX/site-packages` folder
     - Create a folder, for example `yolo_world_efficient_sam`
     - Into this folder: `git clone https://github.com/Curt-Park/yolo-world-with-efficientvit-sam.git`
     - Move to the folder: `yolo_world_with_efficient_sam` where the file `Requirements.txt` is located
     - `pip install -r requirements.txt`
     - In `https://github.com/CVHub520/efficientvit` download the *EfficientViT-L0* model checkpoint and rename it `efficient_SAM_l0.pt` 
     - Place the model in a *models* folder. This folder should be in the folder containing all the other workspaces used in the project
  
</details>

  
<details>
  <summary>
    Step 5: Clone the main branch of this repository.
  </summary>
  
#### TODO
  
  
  
</details>



# Visualization

- ```bash
  ros2 launch igus_rebel_description_ros2 visualize.launch.py load_base:=false mount:=none end_effector:=none camera:=none moveit:=false
  ```
  - The command executes a launch file that visualize the Igus ReBeL robot on RViz2, controllable using the *Joint State Publisher* GUI.
  - The real Igus ReBeL robot can not be controlled using the *Joint State Publisher* GUI, thus the argument `hardware_protocol` should be kept `simulation` (default value).
  - It is possible to change the arguments: `load_base`, `mount`, `end_effector`, `camera`.
  - If camera is inserted, no data will be shown due to the absence of a simulated or real-word environment from which the data comes.
  - The argument `moveit` should be kept `false` to not run MoveIt2 framework for control.
 
    
<br>

- ```bash
  ros2 launch igus_rebel_description_ros2 visualize.launch.py load_base:=false mount:=none camera:=none end_effector:=none load_gazebo:=true moveit:=false
  ```
  - The command executes a launch file that run the simulation on Gazebo Ignition and visualizes the Igus ReBeL robot on RViz2, controllable using the *Joint Position Controller* GUI on Gazebo Ignition.
  - The real Igus ReBeL robot can not be controlled since Gazebo Ignition simulation is used, thus the argument `hardware_protocol` should be kept `simulation` (default value).
  - It is possible to change the arguments: `load_base`, `mount`, `end_effector`, `camera`.
  - If camera is inserted, data will be shown coming from the simulated environment. Change topics if needed.
  - The additional arguments `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` are used to set the position of the spawned robot on Gazebo Ignition.
  - The additional argument `env_gazebo_package` specifies the package where the gazebo world and configuration to use are located. It requires the full name of the package, otherwise it will default to this package.
  - The additional argument `full_world_name` specifies the name of the world file to be loaded in Gazebo Ignition of the type: *name.sdf*.
  - The argument `moveit` should be kept `false` to not run MoveIt2 framework for control.
  - The argument `load_gazebo` must be `true` to run the simulation.

<br>


- ```bash
  ros2 launch igus_rebel_moveit_config demo.launch.py load_base:=false mount:=none camera:=none end_effector:=none
  ```
  - The command executes a launch file that run the Igus ReBeL robot on RViz2, controllable using the MoveIt2 framework directly from RViz2.
  - The real Igus ReBeL robot can be controlled using the MoveIt2 framework commands by changing the arguments to `hardware_protocol:=cri load_base:=true`.
  - It is possible to change the arguments: `load_base`, `mount`, `end_effector`, `camera`.
  - If camera is inserted, no data will be shown due to the absence of a simulated or real-word environment from which the data comes.
  - When running MoveIt2 with a mount and a end effector that is not of that mount and a camera, there is the collision fixed of the camera link. EG v2, realsense and toucher dont enable collision of the camera link. Not a big problem.


<br>

- ```bash
  ros2 launch igus_rebel_moveit_config demo.launch.py load_base:=false mount:=none camera:=none end_effector:=none load_gazebo:=true hardware_protocol:=ignition
  ```
  - The command executes a launch file that run the simulation on Gazebo Ignition and visualizes the Igus ReBeL robot on RViz2, controllable using the the MoveIt2 framework directly from RViz2.
  - The real Igus ReBeL robot can not be controlled since Gazebo Ignition simulation is used, thus the argument `hardware_protocol` should be kept `simulation` (default value).
  - It is possible to change the arguments: `load_base`, `mount`, `end_effector`, `camera`.
  - If camera is inserted, data will be shown coming from the simulated environment. Change topics if needed.
  - The additional arguments `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` are used to set the position of the spawned robot on Gazebo Ignition.
  - The additional argument `env_gazebo_package` specifies the package where the gazebo world and configuration to use are located. It requires the full name of the package, otherwise it will default to this package.
  - The additional argument `full_world_name` specifies the name of the world file to be loaded in Gazebo Ignition of the type: *name.sdf*.
  - The argument `hardware_protocol` must be kept `ignition`.
  - The argument `load_gazebo` must be `true` to run the simulation.



# OctoMap Creation

The first functionality of the architecture is to create and continuously update the occupancy and semantic OctoMap starting from the input data. A decentralized approach is used, based on topic communication between the nodes responsible for various functionalities, such as segmentation, point cloud creation, and OctoMap creation. The execution of these nodes is initiated from a ROS 2 launch file and spun indefinitely until a termination command is run. The methodology is based on four nodes communicating through topics: the sensors node, the segmentation node, the point cloud node, and the OctoMap node.
<br>

- For what concerns the `octomap_normal.launch.py`, the command to execute it is:
  <br>

  ```bash
    ros2 launch fruit_picking_bringup octomap_normal.launch.py run_robot:=true run_rviz:=true run_pt:=true run_octomap:=true
  ```
    - The occupancy octomap is created and updated.
    - Several parameters can be changed in the yaml configuration file ``.
    - The additional arguments `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` are used to set the position of the spawned robot on Gazebo Ignition.
    - Add the extra arguments `load_gazebo:=false test_camera:=true` to start only the realsense and the data streaming.
    - Add the extra arguments `load_gazebo:=false hardware_protocol:=cri moveit:=true` to start the real Igus ReBeL robot and the data streaming.
    - Set `load_base`, `mount`, `camera`, `end_effector` arguments as preferred. In particular with `load_base:=true`, add also the parameter `pointcloud_min_z:=-0.28` in the yaml file.
  <br>

    - Example command to execute the OctoMap creation functionality starting from the data coming from the camera mounted on the simulated Igus ReBeL in Gazebo Ignition:
      <br>

      ```bash
      ros2 launch fruit_picking_bringup octomap_normal.launch.py run_robot:=true run_rviz:=true run_pt:=true run_octomap:=true load_base:=false mount:=mount_v2 camera:=realsense end_effector:=none
      ```
      - It is possible to move the robot using the *Joint Position Controller* GUI in Gazebo Ignition.
        <br>

    - Example command to execute the OctoMap creation functionality starting from the data coming from the Realsense camera runned standalone without real robot:
      <br>

      ```bash
      ros2 launch fruit_picking_bringup octomap_normal.launch.py run_robot:=true run_rviz:=true run_pt:=true run_octomap:=true load_gazebo:=false test_camera:=true
      ```
      

    - Example command to execute the OctoMap creation funcionality starting from the data coming from the Realsense camera mounted on the real Igus ReBeL robot:
      <br>

      ```bash
      ros2 launch fruit_picking_bringup octomap_normal.launch.py run_robot:=true run_rviz:=true run_pt:=true run_octomap:=true load_gazebo:=false hardware_protocol:=cri moveit:=true load_base:=true mount:=mount_v2 camera:=realsense end_effector:=soft_gripper
      ```
      - It is possible to move the robot using the MoveIt2 commands in RViz2.

<br>

- For what concerns the `octomap_segmentation_color_filter.launch.py`, the command to execute it is:
  <br>

  ```bash
    ros2 launch fruit_picking_bringup octomap_segmentation_color_filter.launch.py run_robot:=true run_rviz:=true run_color_filter:=true run_pt:=true run_s_pt:=true run_octomap:=true
  ```

    - The occupancy octomap and the semantic OctoMap is created and updated. The semantic information comes from the color-filtering segmentation node.
    - Several parameters can be changed in the yaml configuration file ``.
    - The additional arguments `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` are used to set the position of the spawned robot on Gazebo Ignition.
    - Add the extra arguments `load_gazebo:=false test_camera:=true` to start only the realsense and the data streaming.
    - Add the extra arguments `load_gazebo:=false hardware_protocol:=cri moveit:=true` to start the real Igus ReBeL robot and the data streaming.
    - Set `load_base`, `mount`, `camera`, `end_effector` arguments as preferred. In particular with `load_base:=true`, add also the parameter `pointcloud_min_z:=-0.28` in the yaml file.
  <br>

    - Example command to execute the OctoMap creation functionality starting from the data coming from the camera mounted on the simulated Igus ReBeL in Gazebo Ignition:
      <br>

      ```bash
      ros2 launch fruit_picking_bringup octomap_segmentation_color_filter.launch.py run_robot:=true run_rviz:=true run_color_filter:=true run_pt:=true run_s_pt:=true run_octomap:=true load_base:=false mount:=mount_v2 camera:=realsense end_effector:=soft_gripper
      ```
      - It is possible to move the robot using the *Joint Position Controller* GUI in Gazebo Ignition.
        <br>

    - Example command to execute the OctoMap creation functionality starting from the data coming from the Realsense camera runned standalone without real robot:
      <br>

      ```bash
      ros2 launch fruit_picking_bringup octomap_segmentation_color_filter.launch.py run_robot:=true run_rviz:=true run_color_filter:=true run_pt:=true run_s_pt:=true run_octomap:=true load_gazebo:=false test_camera:=true
      ```

    - Example command to execute the OctoMap creation funcionality starting from the data coming from the Realsense camera mounted on the real Igus ReBeL robot:
      <br>

      ```bash
      ros2 launch fruit_picking_bringup octomap_segmentation_color_filter.launch.py run_robot:=true run_rviz:=true run_color_filter:=true run_pt:=true run_s_pt:=true run_octomap:=true load_gazebo:=false hardware_protocol:=cri moveit:=true load_base:=true mount:=mount_v2 camera:=realsense end_effector:=soft_gripper
      ```
      - It is possible to move the robot using the MoveIt2 commands in RViz2.

<br>

- For what concerns the `octomap_segmentation_lang_sam.launch.py`, the command to execute it is:
  <br>

  ```bash
    ros2 launch fruit_picking_bringup octomap_segmentation_lang_sam.launch.py run_robot:=true run_rviz:=true run_lang_sam:=true run_pt:=true run_s_pt:=true run_octomap:=true
  ```

    - The occupancy octomap and the semantic OctoMap is created and updated. The semantic information comes from the Lang SAM segmentation node.
    - Several parameters can be changed in the yaml configuration file ``.
    - The additional arguments `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` are used to set the position of the spawned robot on Gazebo Ignition.
    - Add the extra arguments `load_gazebo:=false test_camera:=true` to start only the realsense and the data streaming.
    - Add the extra arguments `load_gazebo:=false hardware_protocol:=cri moveit:=true` to start the real Igus ReBeL robot and the data streaming.
    - Set `load_base`, `mount`, `camera`, `end_effector` arguments as preferred. In particular with `load_base:=true`, add also the parameter `pointcloud_min_z:=-0.28` in the yaml file.
  <br>

    - Example command to execute the OctoMap creation functionality starting from the data coming from the camera mounted on the simulated Igus ReBeL in Gazebo Ignition:
      <br>

      ```bash
      ros2 launch fruit_picking_bringup octomap_segmentation_lang_sam.launch.py run_robot:=true run_rviz:=true run_lang_sam:=true run_pt:=true run_s_pt:=true run_octomap:=true load_base:=false mount:=mount_v2 camera:=realsense end_effector:=soft_gripper spawn_x:=-1.0 spawn_yaw:=0.0 spawn_y:=3.0
      ```
      - It is possible to move the robot using the *Joint Position Controller* GUI in Gazebo Ignition.
        <br>

    - Example command to execute the OctoMap creation functionality starting from the data coming from the Realsense camera runned standalone without real robot:
      <br>

      ```bash
      ros2 launch fruit_picking_bringup octomap_segmentation_lang_sam.launch.py run_robot:=true run_rviz:=true run_lang_sam:=true run_pt:=true run_s_pt:=true run_octomap:=true load_gazebo:=false test_camera:=true
      ```

    - Example command to execute the OctoMap creation funcionality starting from the data coming from the Realsense camera mounted on the real Igus ReBeL robot:
      <br>

      ```bash
      ros2 launch fruit_picking_bringup octomap_segmentation_lang_sam.launch.py run_robot:=true run_rviz:=true run_lang_sam:=true run_pt:=true run_s_pt:=true run_octomap:=true load_gazebo:=false hardware_protocol:=cri moveit:=true load_base:=true mount:=mount_v2 camera:=realsense end_effector:=soft_gripper
      ```
      - It is possible to move the robot using the MoveIt2 commands in RViz2.
     
<br>

- For what concerns the `octomap_segmentation_yolo_world.launch.py`, the command to execute it is:
  <br>

  ```bash
    ros2 launch fruit_picking_bringup octomap_segmentation_yolo_world.launch.py run_robot:=true run_rviz:=true run_yolo_world:=true run_pt:=true run_s_pt:=true run_octomap:=true
  ```

    - The occupancy octomap and the semantic OctoMap is created and updated. The semantic information comes from the YOLO World + EfficientViT SAM segmentation node.
    - Several parameters can be changed in the yaml configuration file ``.
    - The additional arguments `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` are used to set the position of the spawned robot on Gazebo Ignition.
    - Add the extra arguments `load_gazebo:=false test_camera:=true` to start only the realsense and the data streaming.
    - Add the extra arguments `load_gazebo:=false hardware_protocol:=cri moveit:=true` to start the real Igus ReBeL robot and the data streaming.
    - Set `load_base`, `mount`, `camera`, `end_effector` arguments as preferred. In particular with `load_base:=true`, add also the parameter `pointcloud_min_z:=-0.28` in the yaml file.
  <br>

    - Example command to execute the OctoMap creation functionality starting from the data coming from the camera mounted on the simulated Igus ReBeL in Gazebo Ignition:
      <br>

      ```bash
      ros2 launch fruit_picking_bringup octomap_segmentation_yolo_world.launch.py run_robot:=true run_rviz:=true run_yolo_world:=true run_pt:=true run_s_pt:=true run_octomap:=true load_base:=false mount:=mount_v2 camera:=realsense end_effector:=soft_gripper spawn_x:=-1.0 spawn_yaw:=0.0 spawn_y:=3.0
      ```
      - It is possible to move the robot using the *Joint Position Controller* GUI in Gazebo Ignition.
        <br>

    - Example command to execute the OctoMap creation functionality starting from the data coming from the Realsense camera runned standalone without real robot:
      <br>

      ```bash
      ros2 launch fruit_picking_bringup octomap_segmentation_yolo_world.launch.py run_robot:=true run_rviz:=true run_yolo_world:=true run_pt:=true run_s_pt:=true run_octomap:=true load_gazebo:=false test_camera:=true
      ```

    - Example command to execute the OctoMap creation funcionality starting from the data coming from the Realsense camera mounted on the real Igus ReBeL robot:
      <br>

      ```bash
      ros2 launch fruit_picking_bringup octomap_segmentation_yolo_world.launch.py run_robot:=true run_rviz:=true run_yolo_world:=true run_pt:=true run_s_pt:=true run_octomap:=true load_gazebo:=false hardware_protocol:=cri moveit:=true load_base:=true mount:=mount_v2 camera:=realsense end_effector:=soft_gripper
      ```
      - It is possible to move the robot using the MoveIt2 commands in RViz2.

<br>



# Active Vision

The main functionality of the architecture is performing Active Vision for creating a 3D reconstruction of the agricultural environment using a centralized approach. Different from the decentralized approach,
it combines all the functionalities related to the Active Vision into a single node using multi-threading. Regarding the *Active Vision Pipeline Block*, a MultiThreadedExecutor defined in the main node is
used to allow multiple nodes to run in separate threads: *MoveIt2APICreator*, *SegmentationClient*, *PointcloudCreator*, *SegmentedPointcloudCreator*, *ExtendedOctomapCreator*, and *Pipeline*. For what concerns the *Segmentation Block*, an independent client-server node is employed, managed by using a ROS 2 service. Finally, the *Robot Block* consists of executing all the entities related to the Igus ReBeL robot. 

## Simulation (Without mobile base)

- To perform 3D reconstruction using the predefined planning:
   - Run the segmentation server: `ros2 launch fruit_picking_bringup active_vision_predefined_planning_pipeline.launch.py run_yolo_world:=true`;
   - Run the Igus ReBeL robot block: `ros2 launch fruit_picking_bringup active_vision_predefined_planning_pipeline.launch.py run_robot_moveit:=true spawn_x:=-0.7 spawn_yaw:=0.0`;
   - Run the active vision pipeline with predefined planning: `ros2 launch fruit_picking_bringup active_vision_predefined_planning_pipeline.launch.py run_active_vision_pipeline:=true`.
<br>

- To perform 3D reconstruction using the NBV planning:
   - Run the segmentation server: `ros2 launch fruit_picking_bringup active_vision_nbv_planning_pipeline.launch.py run_yolo_world:=true`;
   - Run the Igus ReBeL robot block: `ros2 launch fruit_picking_bringup active_vision_nbv_planning_pipeline.launch.py run_robot_moveit:=true spawn_x:=-0.7 spawn_yaw:=0.0`;
   - Run the active vision pipeline with NBV planning: `ros2 launch fruit_picking_bringup active_vision_nbv_planning_pipeline.launch.py run_active_vision_pipeline:=true`.




## Simulation (With mobile base)

- To perform 3D reconstruction using the predefined planning:
   - Run the segmentation server: `ros2 launch fruit_picking_bringup active_vision_predefined_planning_pipeline.launch.py run_yolo_world:=true`;
   - Run the Igus ReBeL robot block: `ros2 launch fruit_picking_bringup active_vision_predefined_planning_pipeline.launch.py run_robot_moveit:=true spawn_x:=-0.95 spawn_yaw:=0.0 load_base:=true`;
   - Run the active vision pipeline with predefined planning: `ros2 launch fruit_picking_bringup active_vision_predefined_planning_pipeline.launch.py run_active_vision_pipeline:=true load_base:=true`.
<br>

- To perform 3D reconstruction using the NBV planning:
   - Run the segmentation server: `ros2 launch fruit_picking_bringup active_vision_nbv_planning_pipeline.launch.py run_yolo_world:=true`;
   - Run the Igus ReBeL robot block: `ros2 launch fruit_picking_bringup active_vision_nbv_planning_pipeline.launch.py run_robot_moveit:=true spawn_x:=-0.95 spawn_yaw:=0.0 load_base:=true`;
   - Run the active vision pipeline with NBV planning: `ros2 launch fruit_picking_bringup active_vision_nbv_planning_pipeline.launch.py run_active_vision_pipeline:=true load_base:=true`.



## Real-World (With mobile base)

- To perform 3D reconstruction using the predefined planning:
   - Run the segmentation server: `ros2 launch fruit_picking_bringup active_vision_predefined_planning_pipeline.launch.py run_yolo_world:=true`;
   - Run the Igus ReBeL robot block: `ros2 launch fruit_picking_bringup active_vision_predefined_planning_pipeline.launch.py run_robot_moveit:=true load_base:=true hardware_protocol:=cri load_gazebo:=false`;
   - Run the active vision pipeline with predefined planning: `ros2 launch fruit_picking_bringup active_vision_predefined_planning_pipeline.launch.py run_active_vision_pipeline:=true load_base:=true hardware_protocol:=cri load_gazebo:=false`.
<br>

- To perform 3D reconstruction using the NBV planning:
   - Run the segmentation server: `ros2 launch fruit_picking_bringup active_vision_nbv_planning_pipeline.launch.py run_yolo_world:=true`;
   - Run the Igus ReBeL robot block: `ros2 launch fruit_picking_bringup active_vision_nbv_planning_pipeline.launch.py run_robot_moveit:=true load_base:=true hardware_protocol:=cri load_gazebo:=false`;
   - Run the active vision pipeline with NBV planning: `ros2 launch fruit_picking_bringup active_vision_nbv_planning_pipeline.launch.py run_active_vision_pipeline:=true load_base:=true load_gazebo:=false hardware_protocol:=cri`.






[rolling-badge]: https://img.shields.io/badge/-ROLLING-orange?style=flat-square&logo=ros
[rolling]: https://docs.ros.org/en/rolling/index.html
[foxy-badge]: https://img.shields.io/badge/-foxy-orange?style=flat-square&logo=ros
[foxy]: https://docs.ros.org/en/foxy/index.html
[humble-badge]: https://img.shields.io/badge/-HUMBLE-orange?style=flat-square&logo=ros
[humble]: https://docs.ros.org/en/humble/index.html
[iron-badge]: https://img.shields.io/badge/-IRON-orange?style=flat-square&logo=ros
[iron]: https://docs.ros.org/en/iron/index.html
[ubuntu22-badge]: https://img.shields.io/badge/-UBUNTU%2022%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu22]: https://releases.ubuntu.com/jammy/
[ubuntu20-badge]: https://img.shields.io/badge/-UBUNTU%2020%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu20]: https://releases.ubuntu.com/focal/
