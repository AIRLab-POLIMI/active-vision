# av_bringup

The `av_bringup` package is responsible for grouping the two functionalities active vision and 3D reconstruction through OctoMap in useful launch files, as well as a configuration file to set up all the needed parameters.

## Folder Structure

- **config/**: Contains configuration files in YAML format that define parameters for various nodes and functionalities.
- **data/**: Stores data files required for the package, such as the ground truth of the Gazebo simulations.
- **launch/**: Includes launch files written in Python that are used to start different nodes and functionalities.
- **rviz/**: Contains RViz configuration files that define the visualization settings for different pipelines.
