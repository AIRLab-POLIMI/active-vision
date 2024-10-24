
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
     * [Dependencies](#dependencies)
     * [Igus ReBeL](#igus-rebel) 
     * [Gazebo Ignition](#gazebo-ignition)
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
    Step 1: Install the ROS 2 Humble distribution. 
  </summary>
  
- #### Ubuntu 22.04:
  - [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
</details>


<details>
  <summary>
    Step 2: Clone the active vision branch of the <a href="https://github.com/AIRLab-POLIMI/ros2-igus-rebel">Igus ReBeL ROS 2</a> repository.
  </summary>
  
#### TODO
  
  
  
</details>


<details>
  <summary>
    Step 2: Install the dependencies of the Igus ReBeL ROS 2 repository.
  </summary>
  
#### TODO
  
  
  
</details>


  
<details>
  <summary>
    Step 3: Clone the main branch of this repository.
  </summary>
  
#### TODO
  
  
  
</details>


<details>
  <summary>
    Step 2: Install the dependencies of this repository.
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
