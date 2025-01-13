# av_octomap

The `av_octomap` package is responsible for creating and managing the occupancy and semantic OctoMap for the active vision system. This package includes source code, launch files, and configuration files required for the OctoMap creation and management.


## Extended OctoMap Server

This library is the extention of a ROS 2 wrapper that allows the use of the OctoMap functionalities in the ROS 2 system. 

The OctoMap Server2 wrapper comes into the form of a ROS 2 plugin: it can be easily integrated in an application by installing the libraries and importing the executable of this plugin, for example, in a launch file. By launching the plugin, it is possible to visualize using RViz the topics provided by the plugin and see the OctoMap in the form of a marker array for the occupied voxels and another one for the unoccupied voxels. 

To extend the OctoMap Server2 library with additional semantic information, a new package in ROS 2 is created with the exact  same structure as the OctoMap Server2 library, called Extended OctoMap, containing a new plugin. Several modifications to this new plugin have been introduced. The fundamental upgrade is the introduction of an additional hash map where the keys are the addresses of the occupied voxels, and the corresponding values are objects of the ExtendedOctomapData class. This allows extra semantic data to be stored alongside occupancy information.