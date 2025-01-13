# av_segmentation_yolo_world

The `av_segmentation_yolo_world` package is responsible for performing segmentation using the YOLO World model. This package includes source code, launch files, and configuration files required for the segmentation process.

## Key Components

### Launch Files

- **yolo_world_server.launch.py**: This launch file is used to start the YOLO World segmentation server. It includes arguments for customizing the launch process, such as setting the topics for RGB and depth images, camera info, and other parameters. This launch is used for the active vision pipeline.
- **yolo_world.launch.py**: This launch file is used to test the YOLO World segmentation process. It includes arguments for customizing the test process, such as setting the test image and text prompt. This launch is used for the OctoMap creation functionality.

### Source Files

- **yolo_world.py**: This source file implements the `yolo_world.py` class. The class is responsible for performing segmentation using the YOLO World model. It includes methods for subscribing to image and camera info topics, performing segmentation, and publishing the segmented images. This node is used for the OctoMap creation functionality.
- **yolo_world_server.py**: This source file implements the `yolo_world_server.py` class. The class is responsible for handling the server-side operations for the YOLO World segmentation service. It includes methods for processing segmentation requests and sending the responses. This node is used as server in the active vision pipeline, and after receiving the image to segment, it provides the resulting masks.

