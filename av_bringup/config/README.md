# Parameters Configuration

The `parameters.yaml` file in the `av_bringup` package defines various parameters for the active vision system. These parameters include frame IDs, topic names, and settings for different segmentation methods. Below is an explanation of the elements in the `parameters.yaml` file.

## Frames

Defines the frame IDs used in the system.

- **frame_id**: The global frame of reference, typically "world".
- **base_frame_id**: The base frame of the Igus ReBeL robot.
- **realsense_frame_id**: The frame ID for the RealSense camera.
- **realsense_base_frame_id**: The base frame ID for the RealSense camera.
- **gazebo_ignition_camera_frame_id**: The frame ID for the camera in Gazebo Ignition.
- **realsense_camera_frame_id**: The frame ID for the RealSense camera's optical frame.

## Topics

Defines the topics used for input data and segmentation.

### Gazebo Ignition Input Data

- **depth_image_topic**: Topic for the depth image from the virtual camera.
- **depth_image_camera_info_topic**: Topic for the camera info of the depth image.
- **rgb_image_topic**: Topic for the RGB image from the virtual camera.

### RealSense Input Data

- **depth_image_topic**: Topic for the depth image from the RealSense camera.
- **depth_image_camera_info_topic**: Topic for the camera info of the depth image.
- **rgb_image_topic**: Topic for the RGB image from the RealSense camera.

### Segmentation

Defines the topics used for the octomap creation process using different segmentation methods.

#### Color Filter Segmentation

- **rgb_image_topic**: Topic for the RGB image used in color filter segmentation.
- **depth_image_topic**: Topic for the depth image used in color filter segmentation.
- **depth_image_camera_info_topic**: Topic for the camera info of the depth image.
- **tf_topic**: Topic for the transformation frames.
- **octomap_semantic_class_cells_vis_topic**: Topic for visualizing semantic class cells in the OctoMap segmented using the color filtering technique.

#### LANG-SAM Segmentation

- **rgb_image_topic**: Topic for the RGB image used in LANG-SAM segmentation.
- **rgb_images_array_topic**: Topic for the array of RGB images (masks) obtained from the LANG-SAM segmentation.
- **depth_image_topic**: Topic for the depth image used in LANG-SAM segmentation.
- **depth_image_camera_info_topic**: Topic for the camera info of the depth image.
- **confidences_topic**: Topic for the confidence values of the segmentation result.
- **tf_topic**: Topic for the transformation frames.
- **octomap_semantic_class_cells_vis_topic**: Topic for visualizing semantic class cells in the OctoMap segmented using LANG SAM.
- **octomap_confidence_cells_vis_topic**: Topic for visualizing confidence cells in the OctoMap segmented using LANG SAM.
- **octomap_instances_cells_vis_topic**: Topic for visualizing instance cells in the OctoMap segmented using LANG SAM.

#### YOLO World Segmentation

- **rgb_image_topic**: Topic for the RGB image used in YOLO World segmentation.
- **rgb_images_array_topic**: Topic for the array of RGB images (masks) obtained from the YOLO World segmentation.
- **depth_image_topic**: Topic for the depth image used in YOLO World segmentation.
- **depth_image_camera_info_topic**: Topic for the camera info of the depth image.
- **confidences_topic**: Topic for the confidence values of the segmentation result.
- **tf_topic**: Topic for the transformation frames.
- **octomap_semantic_class_cells_vis_topic**: Topic for visualizing semantic class cells in the OctoMap segmented using YOLO World.
- **octomap_confidence_cells_vis_topic**: Topic for visualizing confidence cells in the OctoMap segmented using YOLO World.
- **octomap_instances_cells_vis_topic**: Topic for visualizing instance cells in the OctoMap segmented using YOLO World.

### Pointcloud

Defines the topics used for point cloud data.

- **pointcloud_topic**: Topic containing the full point cloud obtained using the RGB image from the sensor.
- **segmented_pointcloud_topic**: Topic containing the segmented point cloud obtained using the segmentation from the color filtering technique.
- **segmented_pointclouds_array_topic**: Topic containing the array of segmented point clouds obtained using the masks from the LANG SAM or YOLO World segmentation.

### Octomap

Defines the topics used for OctoMap data.

- **octomap_occupied_cells_vis_topic**: Topic for visualizing occupied cells in the OctoMap.
- **octomap_free_cells_vis_topic**: Topic for visualizing free cells in the OctoMap.
- **octomap_occupied_cells_centers_pointcloud_topic**: Topic for the point cloud of occupied cell centers.
- **octomap_binary_topic**: Topic for the binary OctoMap.
- **octomap_full_topic**: Topic for the full OctoMap.
- **octomap_projected_map_topic**: Topic for the 2D projected map of the OctoMap.

## Launch

Defines the parameters for various launch files.

### RealSense Launch

- **enable_rgbd**: Enables RGB-D data.
- **enable_sync**: Enables synchronization.
- **align_depth.enable**: Enables depth alignment.
- **enable_color**: Enables color data.
- **enable_depth**: Enables depth data.
- **pointcloud.enable**: Enables point cloud data.
- **clip_distance**: Sets the clipping distance.

### OctoMap Creation Launch

Defines the parameters for the OctoMap creation process. These paramters are the same for the 3 different process of OctoMap Creation using color filtering, LANG SAM, or YOLO World.

- **resolution**: size, in meters, of a voxel in the OctoMap.
- **height_map**: specifies that the marker array of the OctoMap should be colored with a pattern related to height.
- **colored_map**: specifies that the marker array of the OctoMap should be colored according to the RGB pattern from the original RGB image topic of the
camera.
- **color_factor**: related to the color pattern displayed when the marker array of the OctoMap is height-colored, where lower values produce red-tonality colors and higher values produce blue-tonality colors.
- **filter_ground**: specifies whether to exclude the ground from the OctoMap.
- **filter_speckles**: specifies whether to filter out speckles by removing small groups of isolated occupied cells, although this functionality is currently not used in the code.
- **ground_filter/distance**: distance for ground filtering of the input point cloud.
- **ground_filter/angle**: angle for ground filtering of the input point cloud.
- **ground_filter/plane_distance**: plane distance for ground filtering of the input point cloud.
- **compress_map**: specifies whether to prune nodes in the OctoMap by removing children nodes that have the same occupancy value as their parent.
- **pointcloud_max_x**: Defines the maximum positive value along the x-axis of the world reference system that is considered and inserted into the OctoMap.
- **pointcloud_min_x**: Defines the maximum negative value along the x-axis of the world reference system that is considered and inserted into the OctoMap.
- **pointcloud_max_y**: Defines the maximum positive value along the y-axis of the world reference system that is considered and inserted into the OctoMap.
- **pointcloud_min_y**: Defines the maximum negative value along the y-axis of the world reference system that is considered and inserted into the OctoMap.
- **pointcloud_max_z**: Defines the maximum positive value along the z-axis of the world reference system that is considered and inserted into the OctoMap.
- **pointcloud_min_z**: Defines the maximum negative value along the z-axis of the world reference system that is considered and inserted into the OctoMap.
- **incremental_2D_projection**: enables the incremental update of the 2D projection of the OctoMap.
- **sensor_model/max_range**: defines the distance beyond which a point measurement is considered invalid in the OctoMap calculation.
- **sensor_model/hit**: represents the probability used to update a voxel if it results occupied.
- **sensor_model/miss**: represents the probability used to update a voxel if it results unoccupied.
- **sensor_model/min**: set the minimum log-odds values for a cell or node, corresponding to definite unoccupancy.
- **sensor_model/max**: set the maximum log-odds values for a cell or node, corresponding to definite occupancy.
- **color/r**: Red color value for the occupied cells.
- **color/g**: Green color value for the occupied cells.
- **color/b**: Blue color value for the occupied cells.
- **color/a**: Alpha color value for the occupied cells.
- **color_free/r**: Red color value for free space for the unoccupied cells.
- **color_free/g**: Green color value for free space for the unoccupied cells.
- **color_free/b**: Blue color value for free space for the unoccupied cells.
- **color_free/a**: Alpha color value for free space for the unoccupied cells.
- **process_free_space**: 
- **publish_free_space**: specifies whether to publish the voxels related to unoccupied space, although these are still calculated internally regardless.
- **publish_free_cells**: Enables publishing of free cells.
- **publish_octomap_binary**: Enables publishing of binary OctoMap.
- **publish_octomap_full**: Enables publishing of full OctoMap.
- **publish_centers_pointcloud**: Enables publishing of centers point cloud.
- **publish_2d_projected_map**: Enables publishing of 2D projected map.
- **publish_confidence**: Specifies whether to publish the marker array for visualizing the confidence values in the semantic OctoMap.
- **publish_semantic**: Specifies whether to publish the marker array for visualizing the semantic classes in the OctoMap.
- **publish_instances**: Specifies whether to publish the marker array for visualizing the instances in the semantic OctoMap.
- **segmented_pointcloud_subscription**: 
- **segmented_pointclouds_array_subscription**: 
- **message_filter_queue**: specifies the queue size for the message filters related to the TF.
- **partial_pointcloud_subscription**: Specifies whether to use the segmented-image point cloud instead of the original-image point cloud for the occupancy OctoMap population.
- **segmented_pointclouds_array_subscription**: Specifies whether the node receives the custom message containing the point clouds array, confidences, and semantic class using the zero-shot segmentation approach.
- **segmented_pointcloud_subscription**: Specifies whether the node receives the segmented-image point cloud using the color-filtering approach.
- **segmented_pointcloud_outlier_removal**: Specifies whether to remove outliers from the input point clouds.
- **use_frequency_threshold**: Related to the extended OctoMap update algorithm. More info in the code.
- **frequency_threshold**: Related to the extended OctoMap update algorithm. More info in the code.
- **outlier_detection**: Related to the extended OctoMap update algorithm. More info in the code.
- **search_neighboorhood_ray**: Related to the extended OctoMap update algorithm. More info in the code.
- **correction_neighboorhood_ray**: Related to the extended OctoMap update algorithm. More info in the code.
- **outlier_threshold**: Related to the extended OctoMap update algorithm. More info in the code.
- **weighted_confidence**: Related to the extended OctoMap update algorithm. More info in the code.


### Segmented Point Cloud Creation Launch

Defines the parameters for the creation of the segmented point cloud, used to update the semantic information of the OctoMap.

- **publish_pointclouds_array**: Specifies whether to publish an array of point clouds. It is true when LANG SAM or YOLO World are used. False if color filtering is used.
- **publish_single_pointcloud**: Specifies whether to publish a single point cloud. It is true both with color filtering because the mask of the segmentation is unique, and with LANG SAM or YOLO World, because the unique mask is used for visualization.




### Color Filter Segmentation Launch

Defines the parameters for the color filtering segmentation.

- **publish_original_depth_image**: Specifies whether to publish the original depth image. Used in the decentralized approach.
- **publish_original_depth_image_camera_info**: Specifies whether to publish the camera info for the original depth image. Used in the decentralized approach.
- **publish_original_tf**: Specifies whether to publish the original transformation frames. Used in the decentralized approach.
- **color**: Specifies the color used for segmentation. It can be red, green, and green-red.



### LANG SAM Segmentation Launch

Defines the parameters for the LANG SAM segmentation.

- **approach**: Specifies the approach used for segmentation (e.g., "pub_sub").
- **sam_model_type**: Specifies the type of SAM model used (e.g., "vit_b").
- **publish_masks_array**: Specifies whether to publish an array of segmentation masks.
- **segmentation_prompt**: Specifies the prompt used for segmentation (e.g., "tomato").
- **publish_original_depth_image**: Specifies whether to publish the original depth image. Used in the decentralized approach.
- **publish_original_depth_image_camera_info**: Specifies whether to publish the camera info for the original depth image. Used in the decentralized approach.
- **publish_original_tf**: Specifies whether to publish the original transformation frames. Used in the decentralized approach.



### YOLO World Segmentation Launch

Defines the parameters for the YOLO World segmentation.


- **publish_masks_array**: Specifies whether to publish an array of segmentation masks.
- **publish_original_depth_image**: Specifies whether to publish the original depth image. Used in the decentralized approach.
- **publish_original_depth_image_camera_info**: Specifies whether to publish the camera info for the original depth image. Used in the decentralized approach.
- **publish_original_tf**: Specifies whether to publish the original transformation frames. Used in the decentralized approach.
- **model_path**: Specifies the path to load the model.
- **yolo_world_model_type**: Specifies the type of YOLO World model used (e.g., "yolo_world/l").
- **efficient_SAM_model_type**: Specifies the type of efficient SAM model used (e.g., "l0").
- **segmentation_prompt**: Specifies the prompt used for segmentation (e.g., "tomato").
- **confidence_threshold**: Specifies the confidence threshold for segmentation.
- **nms_threshold**: Specifies the non-maximum suppression threshold for segmentation.
- **confidence_normalization**: Specifies whether to normalize confidence values.



### Active Vision Pipeline Launch


It defines, in addition to the already discussed parameters for the OctoMap creation, other parameters related to the planning of the NBV position of the robot.


- **predefined_planning**: Specifies the predefined planning method (e.g., "zig_zag_plane_narrow_waypoints", "zig_zag_plane_wide_waypoints", "zig_zag_curve_waypoints", "square_plane_wide_waypoints", "square_plane_narrow_waypoints").
- **reconstruction_metric**: Specifies whether to use the reconstruction metric F1 to compare the ground truth and the reconstruction (e.g., "True" for simulation, "False" for real robot).
- **ground_truth_path**: Specifies the path to the ground truth file.
- **step_reconstruction_metric_vis**: Specifies whether to visualize the step-by-step reconstruction metric.
- **candidate_viewpoints_number**: Specifies the number of candidate viewpoints.
- **plane_type_candidate_viewpoints**: Specifies the type of plane for candidate viewpoints (e.g., "square", "circle", "square_random", "circle_random").
- **movement_range**: Specifies the length of the portion containing the candidate viewpoints (e.g., "0.4").
- **orientations**: Specifies the number of different orientations for each candidate (e.g., "4" means up, down, left, right).
- **max_ray_depth**: Specifies the maximum depth for ray casting (e.g., "2.0").
- **ray_step_proportion**: Specifies the multiplier to the OctoMap resolution for defining the ray generation step during non-attention ray casting (e.g., "1.0" means OctoMap resolution).
- **ray_casting_type**: Specifies the type of ray casting (e.g., "full_attention_distance", "full_attention", "attention", "normal", "naive").
- **central_attention_front_distance_ratio**: Specifies the front distance ratio for central attention (e.g., "4.0" for simulation).
- **central_attention_height_distance_ratio**: Specifies the height distance ratio for central attention (e.g., "1.0").
- **central_attention_width_distance_ratio**: Specifies the width distance ratio for central attention (e.g., "4.0" for simulation).
- **ray_casting_vis**: Specifies whether to visualize the ray casting process.
- **utility_type**: Specifies the type of utility used (e.g., "expected_semantic_information_gain").
- **utility_vis**: Specifies whether to visualize the utility.


