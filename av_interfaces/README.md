# av_interfaces

The `av_interfaces` package provides custom message and service definitions for the active vision system. These definitions are used for communication between different nodes in the system, enabling functionalities such as segmentation, point cloud processing, and 3D reconstruction.

## Folder Structure

- **msg/**: Contains custom message definitions used in the active vision system.
  - `ImageArray.msg`: Defines a message for an array of images used for the resulting masks after the segmentation.
  - `PointcloudArray.msg`: Defines a message for an array of point clouds with associated confidences and semantic class, obtained after the projection of the segmentation masks onto the depth image of the scene.
  - `Confidence.msg`: Defines a message for storing confidence values and semantic class information.
- **srv/**: Contains custom service definitions used in the active vision system.
  - `LANGSAMSegmentation.srv`: Defines a service for performing segmentation using the LANG-SAM model.
  - `YOLOWorldSegmentation.srv`: Defines a service for performing segmentation using the YOLO World model.