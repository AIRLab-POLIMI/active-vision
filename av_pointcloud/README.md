# av_pointcloud

The `av_pointcloud` package is responsible for processing and managing point cloud data for the active vision system. This package includes source code, launch files, and configuration files required for creating and handling both full and segmented point clouds.

## Key Components

- **full_pointcloud.hpp**: This header file defines the `FullPointcloud` class, which is responsible for the creation of the point cloud related to the RGB image coming from the sensor. 
- **segmented_pointcloud.hpp**: This header file defines the `SegmentedPointcloud` class, which is responsible for the creation of the point cloud related to the RGB image coming from the segmentation step.
- **full_pointcloud.cpp**: This source file implements the `FullPointcloud` class. The first step of this node is obtaining the depth image, the RGB image and the camera info from the sensor. The depth image is converted into a 3D point cloud based on the depth values. The RGB image is then converted to the appropriate format for integration into the point
cloud.
- **segmented_pointcloud.cpp**: This source file implements the `SegmentedPointcloud` class. This node is designed to
handle both single segmented images, for example the one performed by the color-filtering approach, and arrays of segmented images, such as those provided by Lang SAM or YOLO World + EfficientViT SAM. The first step of this node is obtaining the depth image and camera info from the sensor and the segmented mask in the case of color-filtering or the segmented masks array in the case of the zero-shot segmentation from the node responsible for the segmentation step, together with the confidences and the semantic class. A new pointcloud is created, by combining the depth image and the RBG image(s).
- **main_full_pointcloud.cpp**: This source file contains the main entry point for the full point cloud executable. It initializes the ROS 2 node and starts the full point cloud processing.
- **main_segmented_pointcloud.cpp**: This source file contains the main entry point for the segmented point cloud executable. It initializes the ROS 2 node and starts the segmented point cloud processing.
