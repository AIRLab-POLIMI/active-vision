# av_gazebo_ignition

The `av_gazebo_ignition` package is responsible for providing the environment elements needed for the simulation of the Igus ReBeL robot in a Gazebo Ignition environment. This package includes configuration files, models, and world definitions required for the simulation.

## Folder Structure

- **config/**: Contains configuration files in various formats that define parameters for the Gazebo Ignition simulation environment. The directory includes bridge YAML files for ROS-Gazebo communication and configuration files for the Gazwbo Ignition GUI. In particular:
  - `bridge_description.yaml`: Defines the topics and message types for bridging between ROS and Gazebo Ignition for the robot description.
  - `bridge_moveit.yaml`: Defines the topics and message types for bridging between ROS and Gazebo Ignition for MoveIt integration.
- **models/**: Stores model files required for the simulation, such as ground and tomato plants of different types.
- **worlds/**: Includes world definition files that describe the simulation environment.