#include <av_planning/active_vision_nbv_planning_pipeline.hpp>


namespace active_vision_nbv_planning_pipeline{
    
    /**
     * @brief Constructor for the ActiveVisionNbvPlanningPipeline class.
     * 
     * This constructor initializes the ActiveVisionNbvPlanningPipeline class, setting up various parameters and 
     * configurations required for the next-best-view (NBV) planning pipeline. It initializes the necessary nodes, 
     * reads parameters, and sets up publishers and clients for segmentation and visualization.
     * 
     * Key steps include:
     * 1. **Initialize Nodes**: Store the provided nodes into member variables.
     * 2. **Read Parameters**: Declare and read various parameters required for the NBV planning pipeline.
     * 3. **Initialize Segmentation Service**: Set up the client for the YOLO World segmentation service.
     * 4. **Initialize Publishers**: Create publishers for segmented images, point clouds, and octomap visualizations.
     * 5. **Initialize Octomap**: Set up the octomap data structure and load ground truth data if required.
     * 6. **Initialize MoveIt2 Variables**: Set up initial positions and orientations for MoveIt2.
     * 7. **Set Maximum NBV Planning Steps**: Define the maximum number of steps for the NBV planning process.
     * 
     * @param MoveIt2API_creator Shared pointer to the MoveIt2APIs object.
     * @param pointcloud_creator Shared pointer to the FullPointcloud object.
     * @param segmented_pointcloud_creator Shared pointer to the SegmentedPointcloud object.
     * @param extended_octomap_creator Shared pointer to the ExtendedOctomapServer object.
     * @param segmentationClientNode Shared pointer to the segmentation client node.
     * @param options Node options for ROS 2 node initialization.
     * @param node_name Name of the ROS 2 node.
     */
    ActiveVisionNbvPlanningPipeline::ActiveVisionNbvPlanningPipeline(
        std::shared_ptr<MoveIt2APIs> MoveIt2API_creator,
        std::shared_ptr<full_pointcloud::FullPointcloud> pointcloud_creator,
        std::shared_ptr<segmented_pointcloud::SegmentedPointcloud> segmented_pointcloud_creator,
        std::shared_ptr<extended_octomap_server::ExtendedOctomapServer> extended_octomap_creator,
        std::shared_ptr<rclcpp::Node> segmentationClientNode,
        const rclcpp::NodeOptions &options, 
        const std::string node_name): 
        active_vision_pipeline::ActiveVisionPipeline(
            MoveIt2API_creator, 
            pointcloud_creator, 
            segmented_pointcloud_creator, 
            extended_octomap_creator, 
            segmentationClientNode, 
            options, 
            node_name)
    {
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "NBV planning pipeline constructor started.");


        // Read arguments and save the node into some variables
        MoveIt2API_node_ = MoveIt2API_creator;
        pointcloud_node_ = pointcloud_creator;
        segmented_pointcloud_node_ = segmented_pointcloud_creator;
        extended_octomap_node_ = extended_octomap_creator;
        client_node_ = segmentationClientNode;

        // Read parameters
        hardware_protocol_ = this->declare_parameter("hardware_protocol", "simulation");
        frame_id_ = this->declare_parameter("frame_id", "world");
        base_frame_id_ = this->declare_parameter("base_frame_id", "igus_rebel_base_link");
        queue_size_ = this->declare_parameter<int>("queue_size", 5);
        prompt_ = this->declare_parameter("segmentation_prompt", "tomato");
        confidence_threshold_ = this->declare_parameter<float>("confidence_threshold", 0.001);
        nms_confidence_threshold_ = this->declare_parameter<float>("nms_threshold", 0.2);
        usePartialPointcloud_ = this->declare_parameter("partial_pointcloud_subscription", true); 
        orientations_ = this->declare_parameter<int>("orientations", 0);
        candidateViewpointsNumber_ = this->declare_parameter<int>("candidate_viewpoints_number", 100);
        planeTypeCandidateViewpoints_ = this->declare_parameter("plane_type_candidate_viewpoints", "square");
        movementRange_ = this->declare_parameter<float>("movement_range", 1.0);
        maxRayDepth_ = this->declare_parameter<float>("max_ray_depth", 10.0);
        rayStepProportion_ = this->declare_parameter<float>("ray_step_proportion", 1.0);
        rayCastingType_ = this->declare_parameter("ray_casting_type", "attention");
        centralAttentionFrontDistanceRatio_ = this->declare_parameter<float>("central_attention_front_distance_ratio", 1.0);
        centralAttentionWidthDistanceRatio_ = this->declare_parameter<float>("central_attention_width_distance_ratio", 1.0);
        centralAttentionHeightDistanceRatio_ = this->declare_parameter<float>("central_attention_height_distance_ratio", 1.0);
        rayCastingVis_ = this->declare_parameter("ray_casting_vis", false); 
        utilityType_ = this->declare_parameter("utility_type", "expected_semantic_information_gain");
        utilityVis_ = this->declare_parameter("utility_vis", false); 
        reconstructionMetric_ = this->declare_parameter("reconstruction_metric", false); 
        octree_truth_filename = this->declare_parameter("ground_truth_path", "none");
        stepReconstructionMetricVis_ = this->declare_parameter("step_reconstruction_metric_vis", false); 






        RCLCPP_INFO(this->get_logger(), "Parameters and arguments initialized..");



        // Initialize segmentation service
        this->client_ = client_node_->create_client<av_interfaces::srv::YOLOWorldSegmentation>("/yolo_world_service");
        RCLCPP_INFO(this->get_logger(), "Segmentation client initialized..");


        // Initialize segmented image visualization publisher
        segmentedImagePub_ = create_publisher<Image>("/visualization/yolo_world_segmented_image", rclcpp::SensorDataQoS(rclcpp::KeepLast(3)));

        // Initialize full and segmented pointcloud visualization publisherrclcpp::SensorDataQoS()
        segmentedPointcloudPub_ = create_publisher<PointCloud2>("/visualization/segmented_pointcloud", rclcpp::SensorDataQoS(rclcpp::KeepLast(3)));

        // Initialize octomap data structure
        octree_ = std::make_shared<OcTreeT>(extended_octomap_node_->getRes());
        extendedOctomapMap_ = std::make_shared<ExtendedOctomapMap>();

        // Initialize extended octomap visualization publishers
        extended_octomap_node_->createVisualizations();

        RCLCPP_INFO(this->get_logger(), "Segmented and octomap visual tools initialized.");     


        // Initialize Moveit2 variables
        textPose_.position.x = 0.1;
        textPose_.position.z = 1.1;
        textPose_.orientation.x = 0.0;
        textPose_.orientation.y = 0.0;
        textPose_.orientation.z = 0.0;
        textPose_.orientation.w = 1.0;

        // Load truth octree
        if (reconstructionMetric_){
            RCLCPP_INFO(this->get_logger(), "Loading ground truth octomap data...");

            octree_truth_ = extended_octomap_node_->loadOctree(octree_truth_filename);

            if (octree_truth_ == nullptr) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load ground truth octree from file: %s", octree_truth_filename.c_str());
                // Terminate or return to stop further execution
                rclcpp::shutdown();
            }

            RCLCPP_INFO(this->get_logger(), "Octree truth saved.");
        }

        // Initialize the maximum number of steps for the NBV planning
        maxNBVPlanningSteps_ = 8;   

        
        
    }



    /**
     * @brief Main thread for the NBV planning pipeline.
     * 
     * This function is the main thread for the NBV planning pipeline. It performs the following steps:
     * 1. Initialize the initial position and candidate viewpoints.
     * 2. Visualize the initial position and move the robot to it.
     * 3. Create a data subscriber to obtain data from the robot.
     * 4. Enter a loop to perform NBV planning steps.
     * 5. Obtain data from the robot.
     * 6. Create a segmentation request for the server.
     * 7. Wait for the server to be active.
     * 8. Send the data to the segmentation server.
     * 9. Wait for the response and save the segmented images.
     * 10. Publish the segmented image (for visualization).
     * 11. Create full and segmented point clouds.
     * 12. Update the octomap and publish visualization.
     * 13. Calculate the total entropy of the scene.
     * 14. Calculate the F1 score for the reconstruction.
     * 15. Generate candidate viewpoints.
     * 16. Visualize candidate viewpoints.
     * 17. Calculate valid candidate viewpoints.
     * 18. Visualize valid candidate viewpoints.
     * 19. Choose the next best view (NBV) viewpoint.
     * 20. Visualize the NBV and candidate viewpoints.
     * 21. Move the robot to the NBV.
     * 22. Repeat the loop for the maximum number of NBV planning steps.
     * 23. Output the step entropies and number of instances found with final confidence.
     * 
     */
    void ActiveVisionNbvPlanningPipeline::ActiveVisionNbvPlanningPipelineThread(){
        
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "NBV planning pipeline started.");
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
        
        
        // set the rate for the main thread
	    rclcpp::Rate rate(15);



        // Initialize initial position, candidate viewpoints and NBV pose
        initialPosition_ = getInitialPosition();
        initialPositionCartesian_ = MoveIt2API_node_->fromJointSpaceGoalToCartesianPose(initialPosition_);
        NBV_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
        RCLCPP_INFO(this->get_logger(), "Initial position created.");        




        // Visualize initial position and move to it
        RCLCPP_INFO(this->get_logger(), "Visualize initial position...");

        // Set the fixed frame id of the visualization
        MoveIt2API_node_->visual_tools->setBaseFrame(this->base_frame_id_);

        // Get the cartesian pose of the initial position and publish a marker on it
        MoveIt2API_node_->visual_tools->publishAxisLabeled(
            initialPositionCartesian_, "", rviz_visual_tools::MEDIUM, rviz_visual_tools::GREEN);
        MoveIt2API_node_->visual_tools->trigger();

        RCLCPP_INFO(this->get_logger(), "Moving to initial position..");
        RCLCPP_INFO(this->get_logger(), "Translation: [%f, %f, %f], Rotation (Quaternion): [%f, %f, %f, %f]", 
                initialPositionCartesian_.translation().x(), 
                initialPositionCartesian_.translation().y(), 
                initialPositionCartesian_.translation().z(), 
                Eigen::Quaterniond(initialPositionCartesian_.rotation()).x(), 
                Eigen::Quaterniond(initialPositionCartesian_.rotation()).y(), 
                Eigen::Quaterniond(initialPositionCartesian_.rotation()).z(), 
                Eigen::Quaterniond(initialPositionCartesian_.rotation()).w());

        // bool valid_motion = MoveIt2API_node_->robotPlanAndMove(
        //     eigenIsometry3dToPoseStamped(initialPositionCartesian_), 
        //     "initial_position",
        //     false);
        // if (!valid_motion) {
		// 	RCLCPP_ERROR(this->get_logger(), "Could not move to initial position");
		// 	return;
		// }
        // RCLCPP_INFO(this->get_logger(), "Initial position reached.");

        Eigen::Isometry3d current_pose;
        current_pose = initialPositionCartesian_;



	    // Alternative way: with joint space goal
        bool valid_motion = MoveIt2API_node_->robotPlanAndMove(initialPosition_, "initial_position");
        if (!valid_motion) {
			RCLCPP_ERROR(this->get_logger(), "Could not move to initial position");
			return;
		}
        RCLCPP_INFO(this->get_logger(), "Initial position reached.");


        // Initialize data subscriber
        RCLCPP_INFO(this->get_logger(), "Creating data subscriber...");
        this->createDataSub();
        RCLCPP_INFO(this->get_logger(), "Data subscriber created");

   



        // Create a loop
        for (int i = 0; i < maxNBVPlanningSteps_; ++i) {

            RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
            RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
            RCLCPP_INFO(this->get_logger(), "NBV planning pipeline step %d/%d started.", (i+1), maxNBVPlanningSteps_);
            RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");




            // Obtain data from the robot

            // Lock the variables till the function terminates, locking also the subsciber to save the current data
            std::unique_lock<std::mutex> lock(data_mutex_);
            // Wait intill notification: then check the value of data received
            data_cond_.wait(lock, [this]{ return this->data_received_; });
            data_received_ = false; // Reset the flag

            // Ontain data to send it to the segmentation server
            RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
            RCLCPP_INFO(this->get_logger(), "Getting data from subscriber...");

            Image::ConstSharedPtr working_rgb_msg;
            Image::ConstSharedPtr working_depth_msg;
            CameraInfo::ConstSharedPtr working_camera_info_msg;
            geometry_msgs::msg::TransformStamped::ConstSharedPtr working_tf;

            std::tie(
                working_rgb_msg, working_depth_msg, working_camera_info_msg, working_tf) = std::make_tuple(
                current_rgb_msg_, current_depth_msg_, current_camera_info_msg_, current_tf_);

            RCLCPP_INFO(this->get_logger(), "Data obtained.");
            // Manually release the lock here to allow `saveData` to update the shared data
            lock.unlock();




            // Create segmentation request for the server

            auto request = std::make_shared<av_interfaces::srv::YOLOWorldSegmentation::Request>();
            request->image = *working_rgb_msg;
            request->text_prompt = this->prompt_;
            request->confidence_threshold = this->confidence_threshold_;
            request->nms_threshold = this->nms_confidence_threshold_;




            // Wait for the server to be active

            while (!this->client_->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_WARN(this->get_logger(), "Service not available, waiting again...");
            }

            RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
            RCLCPP_INFO(this->get_logger(), "Sending data to segmentation server...");
            auto result = client_->async_send_request(request);




            // Wait for the response, and save them once received
            RCLCPP_INFO(this->get_logger(), "Waiting data from segmentation server...");
            if (rclcpp::spin_until_future_complete(client_node_, result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = result.get();
                masks_images_array_ = std::make_shared<const ImageArray>(response->masks_images_array);
                merged_masks_image_ = std::make_shared<const Image>(response->merged_masks_images);
                confidences_ = std::make_shared<const Confidence>(response->confidences);
                RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
                RCLCPP_INFO(this->get_logger(), "Segmentation response obtained.");
            }


            // Publish segmented image
            segmentedImagePub_->publish(*merged_masks_image_);



            // Create full and segmented pointcloud
            if (usePartialPointcloud_){
                RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
                RCLCPP_INFO(this->get_logger(), "Creating partial pointcloud...");
                partialPointcloud_ = segmented_pointcloud_node_->imageCb(
                    working_depth_msg, 
                    merged_masks_image_, 
                    working_camera_info_msg);
            } 
            else {
                RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
                RCLCPP_INFO(this->get_logger(), "Creating full pointcloud...");
                fullPointcloud_ = pointcloud_node_->imageCb(
                    working_depth_msg, 
                    working_rgb_msg, 
                    working_camera_info_msg);
            }
            RCLCPP_INFO(this->get_logger(), "Creating segmented pointclouds array...");
            
            segmentedPointcloudArray_ = segmented_pointcloud_node_->imageArrayCb(
                working_depth_msg,
                masks_images_array_,
                working_camera_info_msg,
                confidences_);
            RCLCPP_INFO(this->get_logger(), "Pointclouds created.");



            // Publish full and segmented pointclouds for visualization
            segmentedPointcloud_ = segmented_pointcloud_node_->imageCb(
                working_depth_msg, 
                merged_masks_image_, 
                working_camera_info_msg);
            segmentedPointcloudPub_->publish(*segmentedPointcloud_);

            


            // Update octomap and publish visualization
            RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
            RCLCPP_INFO(this->get_logger(), "Updating octomap...");
            if (usePartialPointcloud_){
                extended_octomap_node_->insertPartialCloudCallback(partialPointcloud_, working_tf);
                extended_octomap_node_->insertSegmentedPointcloudsArrayCallback(segmentedPointcloudArray_, working_tf, partialPointcloud_);
            }
            else {
                extended_octomap_node_->insertCloudCallback(fullPointcloud_);
                extended_octomap_node_->insertSegmentedPointcloudsArrayCallback(segmentedPointcloudArray_, working_tf, fullPointcloud_);
            }
            RCLCPP_INFO(this->get_logger(), "Storing octomap...");
            
            octree_ = extended_octomap_node_->getOcTree();
            extendedOctomapMap_ = extended_octomap_node_->getExtendedOctomapMap();
            
            RCLCPP_INFO(this->get_logger(), "Octomap updated.");




            // Calculate total entropy of the scene
            float stepEntropy = 0.0;

            for (auto it = (*extended_octomap_node_->getExtendedOctomapMap()).begin(); it != (*extended_octomap_node_->getExtendedOctomapMap()).end(); ++it) {
                ExtendedOctomapData& data = it->second;

                if (data.getSemanticClass() == prompt_) {
                    float confidence = data.getConfidence();
                    float key_utility = (-confidence * std::log2(confidence)) - ((1 - confidence) * std::log2(1 - confidence));
                    stepEntropy += key_utility;
                    
                }
            }
            totalEntropies_.push_back(stepEntropy);
            RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------------------------");
            RCLCPP_INFO(this->get_logger(), "The total entropy of the scene after the octomap update is %f", stepEntropy);
            RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------------------------");



            // Calculate F1 score
            if (reconstructionMetric_){
                double currentF1 = reconstructionMetric(stepReconstructionMetricVis_);
                stepF1_.push_back(currentF1);
                RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------------------------");
                RCLCPP_INFO(this->get_logger(), "The current F1 score of the reconstruction is %f", currentF1);
                RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------------------------");
            }



            // Generate candidate viewpoints
            candidateViewpoints_= generatePlaneCandidateViewpoints(
                initialPositionCartesian_, orientations_, planeTypeCandidateViewpoints_, candidateViewpointsNumber_, movementRange_);
            int total_candidates = candidateViewpointsNumber_ + candidateViewpointsNumber_ * orientations_;
            RCLCPP_INFO(this->get_logger(), "%d candidate viewpoints in a %s shape with movement range of %.2f mt created.", total_candidates, planeTypeCandidateViewpoints_.c_str(), movementRange_);        




            // Visualize candidate viewpoints
            RCLCPP_INFO(this->get_logger(), "Visualize candidate viewpoints..");
            MoveIt2API_node_->visual_tools->deleteAllMarkers();
            MoveIt2API_node_->visual_tools->setBaseFrame(this->base_frame_id_);
            MoveIt2API_node_->visual_tools->trigger();

            // Create a vector with all the position names
            std::vector<std::string> positions_names(candidateViewpoints_.size());
            for (size_t i = 0; i < candidateViewpoints_.size(); ++i) {
                positions_names[i] = "pose_" + std::to_string(i);
            }

            // For each pose, get the cartesian pose and publish an arrow on it
            for (size_t i = 0; i < candidateViewpoints_.size(); ++i){
                visualizeArrowPose(candidateViewpoints_[i], 0.1, rviz_visual_tools::YELLOW, rviz_visual_tools::SMALL);

            }
            MoveIt2API_node_->visual_tools->publishText(findUpperCenterPose(candidateViewpoints_), "candidate_viewpoints", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
            MoveIt2API_node_->visual_tools->trigger();
            rclcpp::sleep_for(std::chrono::milliseconds(750));


            


            // Calculate valid candidate viewpoints
            std::vector<Eigen::Isometry3d> validCandidateViewpoints_;

            // For each Eigen pose, get the stamped cartesian pose and check if it is valid
            for (size_t i = 0; i < candidateViewpoints_.size(); ++i){
                if (!MoveIt2API_node_->checkIKSolution(eigenIsometry3dToPoseStamped(candidateViewpoints_[i])->pose)){
                    continue; // The pose can not be reached by the robot
                }
                // The pose is valid and it is added to the valid vector of poses
                validCandidateViewpoints_.push_back(candidateViewpoints_[i]);
            }



            // Visualize valid candidate viewpoints
            RCLCPP_INFO(this->get_logger(), "Visualize valid candidate viewpoints..");
            MoveIt2API_node_->visual_tools->deleteAllMarkers();
            MoveIt2API_node_->visual_tools->setBaseFrame(this->base_frame_id_);
            MoveIt2API_node_->visual_tools->trigger();


            // For each valid candidate Eigen pose, get the stamped cartesian pose and publish an arrow on it
            for (size_t i = 0; i < validCandidateViewpoints_.size(); ++i){
                visualizeArrowPose(validCandidateViewpoints_[i], 0.1, rviz_visual_tools::GREEN, rviz_visual_tools::SMALL);

            }
            MoveIt2API_node_->visual_tools->publishText(findUpperCenterPose(validCandidateViewpoints_), "valid_candidate_viewpoints", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
            MoveIt2API_node_->visual_tools->trigger();


            rclcpp::sleep_for(std::chrono::milliseconds(1500));

            MoveIt2API_node_->visual_tools->deleteAllMarkers();
            MoveIt2API_node_->visual_tools->setBaseFrame(this->base_frame_id_);
            MoveIt2API_node_->visual_tools->trigger();








            // Select the NBV
            RCLCPP_INFO(this->get_logger(), "Choosing NBV viewpoint among the valid candidate viewpoints..");
            NBV_pose_ = chooseNBV(validCandidateViewpoints_, current_pose);
            NBV_pose_ptr_ = eigenIsometry3dToPoseStamped(NBV_pose_);




            // Visualize NBV and candidate viewpoints
            RCLCPP_INFO(this->get_logger(), "Visualize NBV pose and candidate viewpoints..");

            // For each pose except the NBV pose, get the cartesian pose and publish an arrow on it
            for (size_t i = 0; i < validCandidateViewpoints_.size(); ++i){

                geometry_msgs::msg::Point start_point;
                start_point.x = validCandidateViewpoints_[i].translation().x();
                start_point.y = validCandidateViewpoints_[i].translation().y();
                start_point.z = validCandidateViewpoints_[i].translation().z();

                // Assuming the forward direction is correctly represented by transforming the x-axis unit vector by the pose's orientation
                Eigen::Vector3d forward_direction = validCandidateViewpoints_[i].rotation() * Eigen::Vector3d(1, 0, 0);

                // Define the length of the arrow
                double arrow_length = 0.05; // For example, 0.1 meters


                // Check if the current viewpoint is equal to NBV_pose_
                if (validCandidateViewpoints_[i].isApprox(NBV_pose_)) {
                    // Calculate the end point of the arrow based on the forward direction
                    geometry_msgs::msg::Point end_point;
                    end_point.x = start_point.x + forward_direction.x() * arrow_length * 3;
                    end_point.y = start_point.y + forward_direction.y() * arrow_length * 3;
                    end_point.z = start_point.z + forward_direction.z() * arrow_length * 3;

                    MoveIt2API_node_->visual_tools->publishArrow(start_point, end_point, rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);
                }
                else{
                    // Calculate the end point of the arrow based on the forward direction
                    geometry_msgs::msg::Point end_point;
                    end_point.x = start_point.x + forward_direction.x() * arrow_length;
                    end_point.y = start_point.y + forward_direction.y() * arrow_length;
                    end_point.z = start_point.z + forward_direction.z() * arrow_length;

                    MoveIt2API_node_->visual_tools->publishArrow(start_point, end_point, rviz_visual_tools::WHITE, rviz_visual_tools::SMALL);
                }
            }
            std::string nbvUtility = "NBV_pose(entropy=" + std::to_string(maxUtility_) + ")";
            MoveIt2API_node_->visual_tools->publishText(findUpperCenterPose(validCandidateViewpoints_), nbvUtility, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
            MoveIt2API_node_->visual_tools->trigger();


            rclcpp::sleep_for(std::chrono::milliseconds(3000));

            MoveIt2API_node_->visual_tools->deleteAllMarkers();
            MoveIt2API_node_->visual_tools->setBaseFrame(this->base_frame_id_);
            MoveIt2API_node_->visual_tools->trigger();




            // Move to the NBV
            RCLCPP_INFO(this->get_logger(), "Moving to position NBV pose...");
            RCLCPP_INFO(this->get_logger(), "Translation: [%f, %f, %f], Rotation (Quaternion): [%f, %f, %f, %f]", 
                NBV_pose_.translation().x(), 
                NBV_pose_.translation().y(), 
                NBV_pose_.translation().z(), 
                Eigen::Quaterniond(NBV_pose_.rotation()).x(), 
                Eigen::Quaterniond(NBV_pose_.rotation()).y(), 
                Eigen::Quaterniond(NBV_pose_.rotation()).z(), 
                Eigen::Quaterniond(NBV_pose_.rotation()).w());
            valid_motion = MoveIt2API_node_->robotPlanAndMove(NBV_pose_ptr_, "NBV_pose", false);
            if (!valid_motion) {
                RCLCPP_ERROR(this->get_logger(), "Could not move to NBV pose");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "NBV pose reached.");
            current_pose = NBV_pose_;
            


            // Sleep to allow the robot to be fully on position
            RCLCPP_INFO(this->get_logger(), "Robot settlement...");
            rclcpp::sleep_for(std::chrono::milliseconds(2000));






            RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
            RCLCPP_WARN(this->get_logger(), "NBV planning pipeline step terminated.");


        }

        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
        RCLCPP_WARN(this->get_logger(), "NBV planning pipeline terminated.");



        // Results


        // Output step entropies
        for (size_t i = 0; i < totalEntropies_.size(); ++i) {
            RCLCPP_WARN(this->get_logger(), "Entropy of step %zu: %f", i+1, totalEntropies_[i]);
        }



        // Output number of instances found with final confidence
        std::unordered_map<int, float> instanceToConfidence;
        for (auto it = (*extended_octomap_node_->getExtendedOctomapMap()).begin(); it != (*extended_octomap_node_->getExtendedOctomapMap()).end(); ++it) {
            ExtendedOctomapData& data = it->second;
            int instance = data.getInstance();
            float confidence = data.getConfidence();
            
            // Since all elements of the same instance have the same confidence,
            // we can directly assign the confidence to the instance key
            instanceToConfidence[instance] = confidence;
        }
        std::map<int, float> sortedInstanceToConfidence(instanceToConfidence.begin(), instanceToConfidence.end());

        for (const auto& instanceConfidencePair : sortedInstanceToConfidence) {
            if (instanceConfidencePair.first == 0) continue; // Instance 0 refers to not semantic objects
            float instanceEntropy = (-instanceConfidencePair.second * std::log2(instanceConfidencePair.second)) - ((1 - instanceConfidencePair.second) * std::log2(1 - instanceConfidencePair.second));
            RCLCPP_WARN(this->get_logger(), "instance %d has confidence %f and entropy %f", instanceConfidencePair.first, instanceConfidencePair.second, instanceEntropy);
        }


        // Calculate final F2 score for reconstruction
        if (reconstructionMetric_){
            reconstructionMetric(true);
            RCLCPP_WARN(this->get_logger(), "--------------------------------------------------------------------------");
            for (size_t i = 0; i < stepF1_.size(); ++i) {
                RCLCPP_WARN(this->get_logger(), "F1 score of the reconstruction at step %zu: %f", i+1, stepF1_[i]);
            }
            RCLCPP_WARN(this->get_logger(), "--------------------------------------------------------------------------");
        }



    }



    /**
     * @brief Create a data subscriber to obtain data from the robot.
     * 
     * This function creates a data subscriber to obtain data from the robot. It subscribes to the RGB image, depth image, and camera info topics.
     * The subscriber saves the data once it is received and notifies the main thread to continue.
     * 
     */
    void ActiveVisionNbvPlanningPipeline::createDataSub(){
        try {
            data_sync_ = std::make_shared<DataSynchronizer>(DataSyncPolicy(queue_size_), sub_rgb_, sub_depth_, sub_camera_info_);
            data_sync_->registerCallback(
                std::bind(
                    &ActiveVisionNbvPlanningPipeline::saveData,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3));

            sub_rgb_.subscribe(this, "rgb_image");
            sub_depth_.subscribe(this, "depth_image");
            sub_camera_info_.subscribe(this, "depth_image_camera_info");

            // Initialize tf2 buffer and listener
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        } catch (const std::exception& e) {
            // This will catch standard exceptions
            RCLCPP_ERROR(this->get_logger(), "[PredefinedPlanning][createDataSub] %s",e.what());
        } catch (...) {
            // This will catch all other exceptions
            RCLCPP_ERROR(this->get_logger(), "[PredefinedPlanning][createDataSub] Generic error.");
        }        
    }



    /**
     * @brief Save the data once it is received and notify the main thread to continue.
     * 
     * This function saves the data once it is received and notifies the main thread to continue.
     * 
     * @param rgb_msg Shared pointer to the RGB image message.
     * @param depth_msg Shared pointer to the depth image message.
     * @param camera_info_msg Shared pointer to the camera info message.
     */
    void ActiveVisionNbvPlanningPipeline::saveData(
        const Image::ConstSharedPtr & rgb_msg,
        const Image::ConstSharedPtr & depth_msg,
        const CameraInfo::ConstSharedPtr & camera_info_msg)
    {
        // Lock the variables till the function terminates for concurrency purposes
        std::lock_guard<std::mutex> lock(data_mutex_);

        // RCLCPP_DEBUG(this->get_logger(), "Saving data...");

        current_rgb_msg_ = rgb_msg;
        current_depth_msg_ = depth_msg;
        current_camera_info_msg_ = camera_info_msg;

        // Wait for the transform to become available
        std::string target_frame = this->frame_id_;

        try {
            if (this->tf_buffer_->canTransform(
                target_frame, camera_info_msg->header.frame_id, camera_info_msg->header.stamp, rclcpp::Duration::from_seconds(2.0))) 
            {
                current_tf_ = std::make_shared<geometry_msgs::msg::TransformStamped>(
                    this->tf_buffer_->lookupTransform(
                        target_frame, camera_info_msg->header.frame_id, camera_info_msg->header.stamp));
                // Used to tell the main function that is time to execute
                data_received_ = true;
            }
            
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "[PredefinedPlanning][saveData] Could not transform %s to %s: %s", target_frame.c_str(), camera_info_msg->header.frame_id.c_str(), ex.what());
            return;
        } catch (const std::exception& e) {
            // This will catch standard exceptions
            RCLCPP_ERROR(this->get_logger(), "[PredefinedPlanning][saveData] %s",e.what());
        } catch (...) {
            // This will catch all other exceptions
            RCLCPP_ERROR(this->get_logger(), "[PredefinedPlanning][saveData] Generic error.");
        } 
        
        // Used to block the wait, and to check the value of the above flag
        data_cond_.notify_one();

        // RCLCPP_DEBUG(this->get_logger(), "Internal data updated.");
        // From this moment on, when the function terminates (and the scope is destroyed), the lock is released
        // But if no other thread get the lock, again this function will get it one more time since it is called continuosly 
        // by the sync
        // Moreover, the main function can not get the lock if data received is still false. This assures the fact that
        // this function restarts till all the data are obtained (thanks to the bool flag setted if the tf is received)

    }


    
    /**
     * @brief Get the initial position from the config file.
     * 
     * @return The PoseStamped message.
     */
    std::array<double, 6> ActiveVisionNbvPlanningPipeline::getInitialPosition(){

        // Load yaml configuration files
        std::string package_path = ament_index_cpp::get_package_share_directory("av_planning");
        YAML::Node initial_position_file = YAML::LoadFile(package_path + "/config/nbv_initial_position.yaml");
        YAML::Node joint_limits = YAML::LoadFile(package_path + "/config/joint_limits.yaml");


        if (initial_position_file.IsNull()) {
            throw std::runtime_error("Failed to load initial position config file.");
        }
        if (joint_limits.IsNull()) {
            throw std::runtime_error("Failed to load joint limits config file.");
        }

        // Define final array
        std::array<double, 6> initial_position;

        // Define the limit variables and convert degrees to radians
        std::array<float, 6> min_deg, max_deg, min_rad, max_rad;
        for (int i = 0; i < 6; ++i) {
            std::string joint_key = "joint_" + std::to_string(i + 1);
            min_deg[i] = joint_limits[joint_key]["min"].as<float>();
            max_deg[i] = joint_limits[joint_key]["max"].as<float>();
            min_rad[i] = min_deg[i] * M_PI / 180.0;
            max_rad[i] = max_deg[i] * M_PI / 180.0;
        }

        for (int i = 0; i < 6; ++i) {
            std::string joint_key = "joint_" + std::to_string(i + 1);
            float joint_value = initial_position_file["initial_position"][joint_key].as<float>();
            if (joint_value < 0) {
                initial_position[i] = min_rad[i] * joint_value / min_deg[i];
            } else {
                initial_position[i] = max_rad[i] * joint_value / max_deg[i];
            }
        }

        return initial_position;
    }



    /**
     * @brief Generate candidate viewpoints in a plane around the reference pose.
     * 
     * This function generates candidate viewpoints in a plane around the reference pose.
     * The function generates the candidate viewpoints based on the plane type, number of viewpoints, and side length.
     * 
     * @param referencePose The reference pose around which the candidate viewpoints are generated.
     * @param orientations The number of orientations for each candidate viewpoint.
     * @param planeTypeCandidateViewpoints_ The type of plane for the candidate viewpoints.
     * @param N The number of candidate viewpoints.
     * @param sideLength The side length of the plane.
     * @return A vector of Eigen poses representing the candidate viewpoints.
     */
    std::vector<Eigen::Isometry3d> ActiveVisionNbvPlanningPipeline::generatePlaneCandidateViewpoints(
        const Eigen::Isometry3d referencePose, 
        int orientations,
        std::string planeTypeCandidateViewpoints_, 
        int N, 
        float sideLength) 
    {
        std::vector<Eigen::Isometry3d> poses;

        double fixedX = referencePose.translation().x();
        double refY = referencePose.translation().y();
        double refZ = referencePose.translation().z();
        Eigen::Quaterniond referenceOrientation(referencePose.rotation());

        // Create random number generator and distribution outside the lambda
        std::default_random_engine generator(static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count()));
        std::uniform_real_distribution<double> distribution(-sideLength / 10.0, sideLength / 10.0);

        auto addPoseWithOrientationsAndNoise = [&](Eigen::Isometry3d newPose, bool addNoise) {
            if (addNoise) {
                // Generate random noise within +/- 1/10 of the sideLength for each axis
                Eigen::Vector3d noise(0.0, distribution(generator), distribution(generator));
                newPose.translate(noise);
            }

            poses.push_back(newPose);
            if (orientations == 2) {
                Eigen::AngleAxisd rotateUp(M_PI / 6, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd rotateDown(-M_PI / 6, Eigen::Vector3d::UnitY());

                Eigen::Isometry3d newPoseUp = newPose;
                newPoseUp.rotate(rotateUp);

                Eigen::Isometry3d newPoseDown = newPose;
                newPoseDown.rotate(rotateDown);

                poses.push_back(newPoseUp);
                poses.push_back(newPoseDown);
            }
            if (orientations == 4) {
                Eigen::AngleAxisd rotateUp(M_PI / 6, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd rotateDown(-M_PI / 6, Eigen::Vector3d::UnitY());

                // Left and Right rotations
                Eigen::AngleAxisd rotateLeft(M_PI / 6, Eigen::Vector3d::UnitZ());
                Eigen::AngleAxisd rotateRight(-M_PI / 6, Eigen::Vector3d::UnitZ());

                Eigen::Isometry3d newPoseUp = newPose;
                newPoseUp.rotate(rotateUp);

                Eigen::Isometry3d newPoseDown = newPose;
                newPoseDown.rotate(rotateDown);

                Eigen::Isometry3d newPoseLeft = newPose;
                newPoseLeft.rotate(rotateLeft);

                Eigen::Isometry3d newPoseRight = newPose;
                newPoseRight.rotate(rotateRight);

                poses.push_back(newPoseUp);
                poses.push_back(newPoseDown);
                poses.push_back(newPoseLeft);
                poses.push_back(newPoseRight);
            }
        };

        bool addNoise = planeTypeCandidateViewpoints_ == "square_random" || planeTypeCandidateViewpoints_ == "circle_random";

        if (planeTypeCandidateViewpoints_ == "circle" || planeTypeCandidateViewpoints_ == "circle_random") {
            float radius = sideLength;
            int totalRings = std::ceil(std::sqrt(N));
            float deltaRadius = radius / totalRings;

            int totalPosesPlaced = 0;
            for (int ring = 1; ring <= totalRings; ++ring) {
                float currentRadius = ring * deltaRadius;
                float circumference = 2 * M_PI * currentRadius;
                int posesInThisRing = std::round(circumference / deltaRadius);

                if (totalPosesPlaced + posesInThisRing > N) {
                    posesInThisRing = N - totalPosesPlaced;
                }

                float angleIncrement = 2 * M_PI / posesInThisRing;
                for (int i = 0; i < posesInThisRing; ++i) {
                    float angle = i * angleIncrement;
                    float y = refY + currentRadius * cos(angle);
                    float z = refZ + currentRadius * sin(angle);

                    Eigen::Isometry3d newPose = Eigen::Isometry3d::Identity();
                    newPose.translate(Eigen::Vector3d(fixedX, y, z));
                    newPose.rotate(referenceOrientation);

                    addPoseWithOrientationsAndNoise(newPose, addNoise);
                    totalPosesPlaced++;
                    if (totalPosesPlaced >= N) break;
                }
                if (totalPosesPlaced >= N) break;
            }
        } else if (planeTypeCandidateViewpoints_ == "square" || planeTypeCandidateViewpoints_ == "square_random") {
            int numPosesPerSide = std::sqrt(N);
            float spacing = sideLength / (numPosesPerSide - 1);

            float startY = refY - sideLength / 2;
            float startZ = refZ - sideLength / 2;

            for (int i = 0; i < numPosesPerSide; ++i) {
                for (int j = 0; j < numPosesPerSide; ++j) {
                    float y = startY + j * spacing;
                    float z = startZ + i * spacing;

                    Eigen::Isometry3d newPose = Eigen::Isometry3d::Identity();
                    newPose.translate(Eigen::Vector3d(fixedX, y, z));
                    newPose.rotate(referenceOrientation);

                    addPoseWithOrientationsAndNoise(newPose, addNoise);
                }
            }
        }

        return poses;
    }



    /**
     * @brief Find the point representing the uppest and centered point with respect to some poses.
     * 
     * 
     * @param poses The list of viewpoints.
     * @return The uppest centered point.
     */
    geometry_msgs::msg::Pose ActiveVisionNbvPlanningPipeline::findUpperCenterPose(const std::vector<Eigen::Isometry3d>& poses) {
        if (poses.empty()) {
            throw std::invalid_argument("Poses vector is empty.");
        }

        double maxX = poses[0].translation().x(), minX = poses[0].translation().x();
        double maxY = poses[0].translation().y(), minY = poses[0].translation().y();
        double maxZ = poses[0].translation().z();
        double totalDistance = 0.0;
        int count = 0;

        // Find min/max X, Y, and max Z
        for (const auto& pose : poses) {
            const auto& translation = pose.translation();
            maxX = std::max(maxX, translation.x());
            minX = std::min(minX, translation.x());
            maxY = std::max(maxY, translation.y());
            minY = std::min(minY, translation.y());
            maxZ = std::max(maxZ, translation.z());
        }

        // Calculate average distance between poses (assuming uniform distribution)
        int numPosesX = std::round(std::sqrt(poses.size()));
        int numPosesY = numPosesX;
        if (numPosesX > 1) {
            totalDistance += (maxX - minX) / (numPosesX - 1) * (numPosesX - 1);
            totalDistance += (maxY - minY) / (numPosesY - 1) * (numPosesY - 1);
            count = (numPosesX - 1) + (numPosesY - 1);
        }

        double averageDistance = count > 0 ? totalDistance / count : 0.0;

        // Calculate the center of the plane and the new Z coordinate
        double centerX = (maxX + minX) / 2.0;
        double centerY = (maxY + minY) / 2.0;
        double newZ = maxZ + averageDistance * 3;

        // Create the new pose
        geometry_msgs::msg::Pose newPose;
        newPose.position.x = centerX;
        newPose.position.y = centerY;
        newPose.position.z = newZ;
        newPose.orientation.w = 1.0; // Default orientation (no rotation)

        return newPose;
    }



    /**
     * @brief Convert an Eigen::Isometry3d to a geometry_msgs::msg::PoseStamped.
     * 
     * This function converts an Eigen::Isometry3d to a geometry_msgs::msg::PoseStamped.
     * 
     * @param isometry The Eigen::Isometry3d to convert.
     * @return The converted geometry_msgs::msg::PoseStamped.
     */
    geometry_msgs::msg::PoseStamped::SharedPtr ActiveVisionNbvPlanningPipeline::eigenIsometry3dToPoseStamped(const Eigen::Isometry3d& isometry) {
        auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();

        // Set the header
        pose_msg->header.frame_id = base_frame_id_;
        pose_msg->header.stamp = this->now();

        // Set the position
        pose_msg->pose.position.x = isometry.translation().x();
        pose_msg->pose.position.y = isometry.translation().y();
        pose_msg->pose.position.z = isometry.translation().z();

        // Convert Eigen quaternion to ROS 2 message quaternion
        Eigen::Quaterniond q(isometry.rotation());
        pose_msg->pose.orientation.x = q.x();
        pose_msg->pose.orientation.y = q.y();
        pose_msg->pose.orientation.z = q.z();
        pose_msg->pose.orientation.w = q.w();

        return pose_msg;
    }



    /**
     * @brief Visualizes an arrow representing the pose's forward direction in RViz.
     * 
     * This function visualizes an arrow in RViz to represent the forward direction of a given pose. It calculates the start and end points
     * of the arrow based on the pose's translation and orientation, and then publishes the arrow using RViz visual tools.
     * 
     * @param pose The pose to visualize, represented as an Eigen::Isometry3d.
     * @param length The length of the arrow.
     * @param color The color of the arrow.
     * @param scale The scale of the arrow.
     */
    void ActiveVisionNbvPlanningPipeline::visualizeArrowPose(const Eigen::Isometry3d& pose, double length,
        rviz_visual_tools::Colors color, rviz_visual_tools::Scales scale)
    {
        geometry_msgs::msg::Point start_point;
        start_point.x = pose.translation().x();
        start_point.y = pose.translation().y();
        start_point.z = pose.translation().z();

        // Assuming the forward direction is correctly represented by transforming the x-axis unit vector by the pose's orientation
        Eigen::Vector3d forward_direction = pose.rotation() * Eigen::Vector3d(1, 0, 0);

        // Define the length of the arrow
        double arrow_length = length; // For example, 0.1 meters

        // Calculate the end point of the arrow based on the forward direction
        geometry_msgs::msg::Point end_point;
        end_point.x = start_point.x + forward_direction.x() * arrow_length;
        end_point.y = start_point.y + forward_direction.y() * arrow_length;
        end_point.z = start_point.z + forward_direction.z() * arrow_length;

        // Publish the arrow to visualize the pose's forward direction
        MoveIt2API_node_->visual_tools->publishArrow(start_point, end_point, color, scale);
    }



    /**
     * @brief Chooses a random next-best-view (NBV) pose from a given list of poses.
     * 
     * This function selects a random pose from a provided vector of poses. It initializes a random number generator,
     * generates a random index, and returns the pose at that index.
     * 
     * 
     * @param poses A vector of Eigen::Isometry3d poses to choose from.
     * @return The randomly selected Eigen::Isometry3d pose.
     * @throws std::runtime_error if the input vector of poses is empty.
     */
    Eigen::Isometry3d ActiveVisionNbvPlanningPipeline::chooseNBVRandom(const std::vector<Eigen::Isometry3d>& poses) {
        // Check if the input vector is empty
        if (poses.empty()) {
            throw std::runtime_error("Input vector of poses is empty."); // Throw an exception if there are no poses to choose from
        }

        // Initialize a random number generator
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        std::uniform_int_distribution<size_t> distribution(0, poses.size() - 1);

        // Select a random index
        size_t randomIndex = distribution(generator);

        // Return the selected Eigen::Isometry3d
        return poses[randomIndex];
    }



    /**
     * @brief Visualizes a frustum representing the field of view (FOV) in RViz.
     * 
     * This function visualizes a frustum in RViz to represent the field of view of a given pose. It calculates the direction vectors
     * for the frustum sides based on the FOV angles and depth, normalizes these vectors, and then publishes the frustum lines using
     * RViz visual tools.
     * 
     * @param starting_pose The starting pose of the frustum, represented as an Eigen::Isometry3d.
     * @param fov_w_deg The field of view width in degrees.
     * @param fov_h_deg The field of view height in degrees.
     * @param frustum_depth The depth of the frustum.
     */
    void ActiveVisionNbvPlanningPipeline::visualizeFrustum(const Eigen::Isometry3d& starting_pose, double fov_w_deg, double fov_h_deg, double frustum_depth) {
        // Convert FOV from degrees to radians
        double fov_w_rad = fov_w_deg * (M_PI / 180.0);
        double fov_h_rad = fov_h_deg * (M_PI / 180.0);

        // Calculate half-angles for simplicity
        double half_fov_w = fov_w_rad / 2.0;
        double half_fov_h = fov_h_rad / 2.0;

        // Depth is set to 1 meter
        double depth = frustum_depth;

        // Calculate direction vectors for the frustum sides
        std::vector<Eigen::Vector3d> directions = {
            Eigen::Vector3d(1, tan(half_fov_w), tan(half_fov_h)),    // Top right direction
            Eigen::Vector3d(1, -tan(half_fov_w), tan(half_fov_h)),   // Top left direction
            Eigen::Vector3d(1, -tan(half_fov_w), -tan(half_fov_h)),  // Bottom left direction
            Eigen::Vector3d(1, tan(half_fov_w), -tan(half_fov_h))    // Bottom right direction
        };

        // Normalize direction vectors
        for (auto& dir : directions) {
            dir.normalize();
        }

        // Define the actual size of the side of the frustum using Pitagora
        double offset_w = depth * tan(half_fov_w);
        double offset_h = depth * tan(half_fov_h);
        double hypotenuse = sqrt(depth * depth + offset_w * offset_w + offset_h * offset_h);


        // Starting point for the lines is the apex of the frustum
        Eigen::Vector3d starting_point = starting_pose.translation();

        // Visualize the frustum sides
        for (const auto& dir : directions) {
            // Transform direction vector to world frame and scale to desired length (e.g., 1 meter)
            Eigen::Vector3d end_point = starting_point + (starting_pose.rotation() * dir) * hypotenuse;
            MoveIt2API_node_->visual_tools->publishLine(starting_point, end_point, rviz_visual_tools::YELLOW, rviz_visual_tools::XSMALL);
        }

        MoveIt2API_node_->visual_tools->trigger();
    }


    /**
     * @brief Visualizes the base of a frustum representing the field of view (FOV) in RViz.
     * 
     * This function visualizes the base of a frustum in RViz to represent the field of view of a given pose. It calculates the offsets
     * for the frustum corners based on the FOV angles and depth, transforms these corners to the world frame, and then publishes the
     * frustum base as a rectangle using RViz visual tools.
     * 
     * @param starting_pose The starting pose of the frustum, represented as an Eigen::Isometry3d.
     * @param fov_w_deg The field of view width in degrees.
     * @param fov_h_deg The field of view height in degrees.
     * @param frustum_depth The depth of the frustum.
     */
    void ActiveVisionNbvPlanningPipeline::visualizeFrustumBase(const Eigen::Isometry3d& starting_pose, double fov_w_deg, double fov_h_deg, double frustum_depth) {
        // Convert FOV from degrees to radians
        double fov_h_rad = fov_h_deg * (M_PI / 180.0);
        double fov_w_rad = fov_w_deg * (M_PI / 180.0);

        // Calculate half-angles for simplicity
        double half_fov_h = fov_h_rad / 2.0;
        double half_fov_w = fov_w_rad / 2.0;

        double depth = frustum_depth;

        // Calculate the offsets at the depth (the distance between the center of the plane and the corner defined by the angle)
        double offset_h = depth * tan(half_fov_h);
        double offset_w = depth * tan(half_fov_w);

        // Define frustum base corners in camera frame
        std::vector<Eigen::Vector3d> corners = {
            Eigen::Vector3d(depth, offset_w, offset_h),    // Top right
            Eigen::Vector3d(depth, -offset_w, offset_h),   // Top left
            Eigen::Vector3d(depth, -offset_w, -offset_h),  // Bottom left
            Eigen::Vector3d(depth, offset_w, -offset_h)    // Bottom right
        };

        // Transform corners to world frame
        for (auto& corner : corners) {
            corner = starting_pose * corner;
        }

        // Visualize the base of the frustum as a rectangle
        for (size_t i = 0; i < corners.size(); ++i) {
            MoveIt2API_node_->visual_tools->publishLine(corners[i], corners[(i + 1) % corners.size()], rviz_visual_tools::RED, rviz_visual_tools::XSMALL);
        }

        MoveIt2API_node_->visual_tools->trigger();

        
    }


    
    /**
     * @brief Generates a grid of points on the base of a frustum representing the field of view (FOV).
     * 
     * This function generates a grid of points on the base of a frustum representing the field of view (FOV). It calculates the offsets
     * for the frustum corners based on the FOV angles and depth, and then generates a grid of points on the frustum base using the
     * resolution provided.
     * 
     * @param starting_pose The starting pose of the frustum, represented as an Eigen::Isometry3d.
     * @param fov_w_deg The field of view width in degrees.
     * @param fov_h_deg The field of view height in degrees.
     * @param resolution_m The resolution of the grid in meters.
     * @param frustum_depth The depth of the frustum.
     * @return A vector of Eigen::Vector3d points on the frustum base.
     */
    std::vector<Eigen::Vector3d> ActiveVisionNbvPlanningPipeline::generateFrustumBaseGrid(const Eigen::Isometry3d& starting_pose, double fov_w_deg, double fov_h_deg, double resolution_m, double frustum_depth) {
        // Convert FOV from degrees to radians
        double fov_h_rad = fov_h_deg * (M_PI / 180.0);
        double fov_w_rad = fov_w_deg * (M_PI / 180.0);

        // Calculate half-angles for simplicity
        double half_fov_h = fov_h_rad / 2.0;
        double half_fov_w = fov_w_rad / 2.0;

        // Depth is set to frustum_depth
        double depth = frustum_depth;

        // Calculate the offsets at the depth
        double offset_h = depth * tan(half_fov_h);
        double offset_w = depth * tan(half_fov_w);

        // Calculate the number of steps based on the resolution
        int w = static_cast<int>(std::ceil((2 * offset_w) / resolution_m));
        int h = static_cast<int>(std::ceil((2 * offset_h) / resolution_m));

        // Adjust step sizes based on the new number of steps
        double step_w = (2 * offset_w) / (w - 1);
        double step_h = (2 * offset_h) / (h - 1);

        std::vector<Eigen::Vector3d> gridPoints;
        gridPoints.reserve(h * w);

        // Generate grid points
        for (int i = 0; i < h; ++i) {
            for (int j = 0; j < w; ++j) {
                // Calculate interpolation factors for width and height
                double t_w = j * step_w - offset_w;
                double t_h = i * step_h - offset_h;

                // Interpolate to find the point on the frustum base
                Eigen::Vector3d point = starting_pose * Eigen::Vector3d(depth, t_w, t_h);
                gridPoints.push_back(point);
            }
        }

        return gridPoints;
    }



    /**
     * @brief Perform naive ray casting to find the closest occupied voxel to the starting point.
     * 
     * This function performs naive ray casting to find the closest occupied voxel to the starting point. It generates ray end points
     * at a large-enough distance from the starting point to cover the common possible front area, and then casts rays to find the closest
     * occupied voxel to the starting point. This approach ensures that each ray is cast within the FOV from the starting point to a enough-distant ending point, 
     * effectively covering the common possible front area.
     * 
     * @param pose The starting pose for ray casting, represented as an Eigen::Isometry3d.
     * @param fov_w The field of view width in degrees.
     * @param fov_h The field of view height in degrees.
     * @param visualization A boolean flag to enable visualization of the ray end points.
     * @return A set of octomap::OcTreeKey keys representing the occupied voxels found by ray casting.
     */
    octomap::KeySet ActiveVisionNbvPlanningPipeline::performNaiveRayCasting(Eigen::Isometry3d pose, double fov_w, double fov_h, bool visualization=false){

        octomap::KeySet finalSet;
        double ray_depth = maxRayDepth_;
        std::vector<Eigen::Vector3d> end_points = generateFrustumBaseGrid(pose, fov_w, fov_h, extended_octomap_node_->getRes(), ray_depth);

        if (visualization){
            // Visualize ray's end point based on the current octomap limit (set in the parameter file)
            for (auto end: end_points){
                MoveIt2API_node_->visual_tools->publishSphere(end, rviz_visual_tools::GREEN, rviz_visual_tools::MEDIUM);
                MoveIt2API_node_->visual_tools->trigger();
                rclcpp::sleep_for(std::chrono::milliseconds(2));
            }
        }


        octomap::KeyRay voxelRayKeys; 
        octomap::point3d starting_3d_point(pose.translation().x(), pose.translation().y(), pose.translation().z());

        for (auto& end_point: end_points){

            // Save the voxel keys traversed by the ray
            octomap::point3d ending_3d_point(end_point.x(), end_point.y(), end_point.z());
            if (!octree_->computeRayKeys(starting_3d_point, ending_3d_point, voxelRayKeys)) {
                throw std::runtime_error("Could not perform ray casting");
            }
            // Find the key of the closest voxel to the starting point that is occupied
            octomap::OcTreeKey closestOccupiedKey;
            double minDistance = std::numeric_limits<double>::max();
            for (auto& key: voxelRayKeys){
                // It could happen that a key dont exists in the octree, since in some configurations
                // it can contain only occupied cells. A cehck is needed
                octomap::OcTreeNode* node = octree_->search(key);
                if (node && octree_->isNodeOccupied(node)){ // Check if node is not null and is occupied
                    octomap::point3d keyPoint = octree_->keyToCoord(key);
                    double distance = starting_3d_point.distance(keyPoint);
                    if (distance < minDistance){
                        minDistance = distance;
                        closestOccupiedKey = key;
                    }
                }
            }
            
            // Insert the key into the main keyset
            finalSet.insert(closestOccupiedKey);

        }
        return finalSet;
    }



    /**
     * @brief Generates direction vectors for the base of a frustum representing the field of view (FOV).
     * 
     * This function generates direction vectors for the base of a frustum in RViz to represent the field of view of a given pose.
     * It calculates the offsets for the frustum corners based on the FOV angles and voxel resolution, transforms these directions to the
     * world frame, and returns the direction vectors.
     * 
     * @param starting_pose The starting pose of the frustum, represented as an Eigen::Isometry3d.
     * @param fov_w_deg The field of view width in degrees.
     * @param fov_h_deg The field of view height in degrees.
     * @param resolution_m The resolution in meters.
     * @return A vector of Eigen::Vector3d direction vectors for the frustum base.
     */
    std::vector<Eigen::Vector3d> ActiveVisionNbvPlanningPipeline::generateFrustumBaseDirections(const Eigen::Isometry3d& starting_pose, double fov_w_deg, double fov_h_deg, double resolution_m) {
        // Convert FOV from degrees to radians
        double fov_h_rad = fov_h_deg * (M_PI / 180.0);
        double fov_w_rad = fov_w_deg * (M_PI / 180.0);

        // Calculate half-angles for simplicity
        double half_fov_h = fov_h_rad / 2.0;
        double half_fov_w = fov_w_rad / 2.0;

        // Calculate the offsets at a unit depth, since we're interested in directions
        double offset_h = tan(half_fov_h);
        double offset_w = tan(half_fov_w);

        // Calculate the number of steps based on the resolution
        int w = static_cast<int>(std::ceil((2 * offset_w) / resolution_m));
        int h = static_cast<int>(std::ceil((2 * offset_h) / resolution_m));

        // Adjust step sizes based on the new number of steps
        double step_w = (2 * offset_w) / (w - 1);
        double step_h = (2 * offset_h) / (h - 1);

        std::vector<Eigen::Vector3d> directions;
        directions.reserve(h * w);

        // Generate direction vectors
        for (int i = 0; i < h; ++i) {
            for (int j = 0; j < w; ++j) {
                // Calculate interpolation factors for width and height
                double t_w = j * step_w - offset_w;
                double t_h = i * step_h - offset_h;

                // Calculate direction vector
                Eigen::Vector3d direction = Eigen::Vector3d(1, t_w, t_h).normalized();
                directions.push_back(starting_pose.rotation() * direction); // Apply rotation from starting pose
            }
        }

        return directions;
    }



    /**
     * @brief Calculates the center of the occupied nodes in the occupancy map.
     * 
     * This function iterates through all the leaf nodes in the octree, accumulates the coordinates of the occupied nodes,
     * and calculates the average coordinates to determine the center of the occupied nodes in the occupancy map.
     * 
     * @return The center of the occupied nodes as an Eigen::Vector3d.
     */
    Eigen::Vector3d ActiveVisionNbvPlanningPipeline::getOccupancyMapCenter() {
        double sumX = 0, sumY = 0, sumZ = 0; // Accumulators for the sum of coordinates
        unsigned int count = 0; // Count of occupied nodes

        // Iterate through all leaf nodes
        for (auto it = octree_->begin(octree_->getTreeDepth()), end = octree_->end(); it != end; ++it) {
            if (octree_->isNodeOccupied(*it)) {
                // Accumulate the coordinates
                sumX += it.getX();
                sumY += it.getY();
                sumZ += it.getZ();
                ++count; // Increment the count
            }
        }

        if (count > 0) {
            // Calculate the average for each coordinate axis to get the center
            double avgX = sumX / count;
            double avgY = sumY / count;
            double avgZ = sumZ / count;
            // Return the average coordinates as an Eigen::Vector3d
            return Eigen::Vector3d(avgX, avgY, avgZ);
        } else {
            // Return a default Eigen::Vector3d if there are no occupied nodes
            return Eigen::Vector3d(0, 0, 0);
        }
    }


    /**
     * @brief Finds the furthest occupied point in the octree from the current pose.
     * 
     * This function iterates through all the leaf nodes in the octree, transforms the coordinates of the occupied nodes to the local frame
     * of the current pose, and finds the furthest point along the x-axis. It then transforms this point back to the global frame and returns it.
     * 
     * @param current_pose The current pose represented as an Eigen::Isometry3d.
     * @return The furthest occupied point as an Eigen::Vector3d.
     */
    Eigen::Vector3d ActiveVisionNbvPlanningPipeline::findFurthestPoint(const Eigen::Isometry3d& current_pose) {
        Eigen::Vector3d rightmostPoint = Eigen::Vector3d(-std::numeric_limits<double>::infinity(), 0, 0);
        Eigen::Vector3d localPoint;

        for (auto it = octree_->begin(octree_->getTreeDepth()), end = octree_->end(); it != end; ++it) {
            if (octree_->isNodeOccupied(*it)) {
                // Transform global coordinates to local frame
                localPoint = current_pose.inverse() * Eigen::Vector3d(it.getX(), it.getY(), it.getZ());

                // Update rightmost point
                if (localPoint.x() > rightmostPoint.x()) {
                    rightmostPoint = localPoint;
                }
            }
        }

        // Transform the rightmost point back to the global frame
        if (rightmostPoint.x() != -std::numeric_limits<double>::infinity()) {
            return current_pose * rightmostPoint;
        }

        // Return an invalid point if no occupied nodes were found
        return Eigen::Vector3d::Zero();
    }



    /**
     * @brief Finds the closest occupied point in the octree to the current pose.
     * 
     * This function iterates through all the leaf nodes in the octree, transforms the coordinates of the occupied nodes to the local frame
     * of the current pose, and finds the closest point along the x-axis. It then transforms this point back to the global frame and returns it.
     * 
     * @param current_pose The current pose represented as an Eigen::Isometry3d.
     * @return The closest occupied point as an Eigen::Vector3d.
     */
    Eigen::Vector3d ActiveVisionNbvPlanningPipeline::findClosestPoint(const Eigen::Isometry3d& current_pose) {
        Eigen::Vector3d leftmostPoint = Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0, 0);
        Eigen::Vector3d localPoint;

        for (auto it = octree_->begin(octree_->getTreeDepth()), end = octree_->end(); it != end; ++it) {
            if (octree_->isNodeOccupied(*it)) {
                // Transform global coordinates to local frame
                localPoint = current_pose.inverse() * Eigen::Vector3d(it.getX(), it.getY(), it.getZ());

                // Update leftmost point
                if (localPoint.x() < leftmostPoint.x()) {
                    leftmostPoint = localPoint;
                }
            }
        }

        // Transform the leftmost point back to the global frame
        if (leftmostPoint.x() != std::numeric_limits<double>::infinity()) {
            return current_pose * leftmostPoint;
        }

        // Return an invalid point if no occupied nodes were found
        return Eigen::Vector3d::Zero();
    }



    /**
     * @brief Finds the highest occupied point in the octree from the current pose.
     * 
     * This function iterates through all the leaf nodes in the octree, transforms the coordinates of the occupied nodes to the local frame
     * of the current pose, and finds the highest point along the z-axis. It then transforms this point back to the global frame and returns it.
     * 
     * @param current_pose The current pose represented as an Eigen::Isometry3d.
     * @return The highest occupied point as an Eigen::Vector3d.
     */
    Eigen::Vector3d ActiveVisionNbvPlanningPipeline::findHighestPoint(const Eigen::Isometry3d& current_pose) {
        Eigen::Vector3d aheadPoint = Eigen::Vector3d(0, 0, -std::numeric_limits<double>::infinity());
        Eigen::Vector3d localPoint;

        for (auto it = octree_->begin(octree_->getTreeDepth()), end = octree_->end(); it != end; ++it) {
            if (octree_->isNodeOccupied(*it)) {
                // Transform global coordinates to local frame
                localPoint = current_pose.inverse() * Eigen::Vector3d(it.getX(), it.getY(), it.getZ());

                // Update ahead point
                if (localPoint.z() > aheadPoint.z()) {
                    aheadPoint = localPoint;
                }
            }
        }

        // Transform the ahead point back to the global frame
        if (aheadPoint.z() != -std::numeric_limits<double>::infinity()) {
            return current_pose * aheadPoint;
        }

        // Return an invalid point if no occupied nodes were found
        return Eigen::Vector3d::Zero();
    }



    /**
     * @brief Finds the lowest occupied point in the octree from the current pose.
     * 
     * This function iterates through all the leaf nodes in the octree, transforms the coordinates of the occupied nodes to the local frame
     * of the current pose, and finds the lowest point along the z-axis. It then transforms this point back to the global frame and returns it.
     * 
     * @param current_pose The current pose represented as an Eigen::Isometry3d.
     * @return The lowest occupied point as an Eigen::Vector3d.
     */
    Eigen::Vector3d ActiveVisionNbvPlanningPipeline::findLowestPoint(const Eigen::Isometry3d& current_pose) {
        Eigen::Vector3d behindPoint = Eigen::Vector3d(0, 0, std::numeric_limits<double>::infinity());
        Eigen::Vector3d localPoint;

        for (auto it = octree_->begin(octree_->getTreeDepth()), end = octree_->end(); it != end; ++it) {
            if (octree_->isNodeOccupied(*it)) {
                // Transform global coordinates to local frame
                localPoint = current_pose.inverse() * Eigen::Vector3d(it.getX(), it.getY(), it.getZ());

                // Update behind point
                if (localPoint.z() < behindPoint.z()) {
                    behindPoint = localPoint;
                }
            }
        }

        // Transform the behind point back to the global frame
        if (behindPoint.z() != std::numeric_limits<double>::infinity()) {
            return current_pose * behindPoint;
        }

        // Return an invalid point if no occupied nodes were found
        return Eigen::Vector3d::Zero();
    }



    /**
     * @brief Finds the leftmost occupied point in the octree from the current pose.
     * 
     * This function iterates through all the leaf nodes in the octree, transforms the coordinates of the occupied nodes to the local frame
     * of the current pose, and finds the leftmost point along the y-axis. It then transforms this point back to the global frame and returns it.
     * 
     * @param current_pose The current pose represented as an Eigen::Isometry3d.
     * @return The leftmost occupied point as an Eigen::Vector3d.
     */
    Eigen::Vector3d ActiveVisionNbvPlanningPipeline::findLeftMostPoint(const Eigen::Isometry3d& current_pose) {
        Eigen::Vector3d highestPoint(0, -std::numeric_limits<double>::infinity(), 0);
        Eigen::Vector3d localPoint;

        for (auto it = octree_->begin(octree_->getTreeDepth()), end = octree_->end(); it != end; ++it) {
            if (octree_->isNodeOccupied(*it)) {
                // Transform global coordinates to local frame
                localPoint = current_pose.inverse() * Eigen::Vector3d(it.getX(), it.getY(), it.getZ());

                // Update highest point
                if (localPoint.y() > highestPoint.y()) {
                    highestPoint = localPoint;
                }
            }
        }

        // Transform the highest point back to the global frame
        if (highestPoint.y() != -std::numeric_limits<double>::infinity()) {
            return current_pose * highestPoint;
        }

        // Return an invalid point if no occupied nodes were found
        return Eigen::Vector3d::Zero();
    }



    /**
     * @brief Finds the rightmost occupied point in the octree from the current pose.
     * 
     * This function iterates through all the leaf nodes in the octree, transforms the coordinates of the occupied nodes to the local frame
     * of the current pose, and finds the rightmost point along the y-axis. It then transforms this point back to the global frame and returns it.
     * 
     * @param current_pose The current pose represented as an Eigen::Isometry3d.
     * @return The rightmost occupied point as an Eigen::Vector3d.
     */
    Eigen::Vector3d ActiveVisionNbvPlanningPipeline::findRightMostPoint(const Eigen::Isometry3d& current_pose) {
        Eigen::Vector3d lowestPoint(0, std::numeric_limits<double>::infinity(), 0);
        Eigen::Vector3d localPoint;

        for (auto it = octree_->begin(octree_->getTreeDepth()), end = octree_->end(); it != end; ++it) {
            if (octree_->isNodeOccupied(*it)) {
                // Transform global coordinates to local frame
                localPoint = current_pose.inverse() * Eigen::Vector3d(it.getX(), it.getY(), it.getZ());

                // Update lowest point
                if (localPoint.y() < lowestPoint.y()) {
                    lowestPoint = localPoint;
                }
            }
        }

        // Transform the lowest point back to the global frame
        if (lowestPoint.y() != std::numeric_limits<double>::infinity()) {
            return current_pose * lowestPoint;
        }

        // Return an invalid point if no occupied nodes were found
        return Eigen::Vector3d::Zero();
    }



    /**
     * @brief Retrieves the positions of points in the octomap that match the target semantic class.
     * 
     * This function iterates through all entries in the extended octomap map, checks if the semantic class of each entry matches
     * the target class, and retrieves the 3D positions of matching points. Optionally, it visualizes the points in RViz.
     * 
     * @param visualization A boolean flag to enable or disable visualization in RViz.
     * @return A vector of Eigen::Vector3d positions of points that match the target semantic class.
     */
    std::vector<Eigen::Vector3d> ActiveVisionNbvPlanningPipeline::getSemanticArea(bool visualization=false) {
        std::vector<Eigen::Vector3d> target_positions;

        // Use an iterator to iterate through all entries in the extended octomap map
        for (auto it = extendedOctomapMap_->begin(); it != extendedOctomapMap_->end(); ++it) {
            const octomap::OcTreeKey& key = it->first;
            ExtendedOctomapData& data = it->second;

            // Check if the semantic class matches the target
            if (data.getSemanticClass() == prompt_) {
                // Retrieve the 3D position using the octree
                octomap::point3d point = octree_->keyToCoord(key);
                Eigen::Vector3d point_eigen(point.x(), point.y(), point.z());

                if (rayCastingType_ == "full_attention" || rayCastingType_ == "full_attention_distance"){
                    if (point_eigen.x() <= newFurthest_ && point_eigen.x() >= newClosest_ &&
                    point_eigen.z() <= newHighest_ && point_eigen.z() >= newLowest_ &&
                    point_eigen.y() <= newLeftMost_ && point_eigen.y() >= newRightMost_) {
                        target_positions.push_back(point_eigen);
                        if (visualization){
                            MoveIt2API_node_->visual_tools->publishSphere(point_eigen, rviz_visual_tools::RED, rviz_visual_tools::SMALL);
                        }
                    }
                }
                else {
                    target_positions.push_back(point_eigen);

                }
            }
        }
        if (visualization){
            MoveIt2API_node_->visual_tools->trigger();
        }
        return target_positions;
    }


    /**
     * @brief Generates direction vectors for the base of a frustum representing the field of view (FOV) with attention to specific target points.
     * 
     * This function generates direction vectors for the base of a frustum in RViz to represent the field of view of a given pose, focusing on specific
     * target points that match a semantic class. It calculates the offsets for the frustum corners based on the FOV angles, transforms these directions
     * to the world frame, and returns the direction vectors. Optionally, it visualizes the points and directions in RViz.
     * 
     * @param starting_pose The starting pose of the frustum, represented as an Eigen::Isometry3d.
     * @param fov_w_deg The field of view width in degrees.
     * @param fov_h_deg The field of view height in degrees.
     * @param visualization A boolean flag to enable or disable visualization in RViz.
     * @return A vector of Eigen::Vector3d direction vectors for the frustum base.
     */
    std::vector<Eigen::Vector3d> ActiveVisionNbvPlanningPipeline::generateFrustumBaseAttentionDirections(const Eigen::Isometry3d& starting_pose, double fov_w_deg, double fov_h_deg, bool visualization=false) {
        // Convert FOV from degrees to radians
        double fov_h_rad = fov_h_deg * (M_PI / 180.0);
        double fov_w_rad = fov_w_deg * (M_PI / 180.0);

        // Calculate half-angles for simplicity
        double half_fov_h = fov_h_rad / 2.0;
        double half_fov_w = fov_w_rad / 2.0;

        // Calculate the dimensions of the projection plane at distance d=1 (simplified)
        double projection_plane_h = 2 * tan(half_fov_h);
        double projection_plane_w = 2 * tan(half_fov_w);

        std::vector<Eigen::Vector3d> directions;
        std::vector<Eigen::Vector3d> target_positions = getSemanticArea(visualization);

        // Forward vector (n^) of the starting pose
        Eigen::Vector3d n_hat = starting_pose.rotation() * Eigen::Vector3d(1, 0, 0);
        // Right vector (u^) of the starting pose
        Eigen::Vector3d u_hat = starting_pose.rotation() * Eigen::Vector3d(0, 1, 0);
        // Up vector (v^) of the starting pose
        Eigen::Vector3d v_hat = starting_pose.rotation() * Eigen::Vector3d(0, 0, 1);

        for (const auto& target : target_positions) {
            Eigen::Vector3d direction = (target - starting_pose.translation());

            // Step 1: Calculate d' = p⃗ ⋅ n^
            double d_prime = direction.dot(n_hat);
            if (d_prime <= 0) {
                // Point is behind the camera
                continue;
            }

            // Step 2: Calculate p⃗ ′ = d * (1/d' * p⃗ - n^)
            Eigen::Vector3d p_prime = (direction / d_prime) - n_hat;

            // Step 3: Calculate u′ = p⃗ ′ ⋅ u^ and check horizontal limits
            double u_prime = p_prime.dot(u_hat);
            if (u_prime < -projection_plane_w / 2 || u_prime > projection_plane_w / 2) {
                // Point is outside the horizontal FOV
                continue;
            }

            // Step 4: Calculate v′ = p⃗ ′ ⋅ v^ and check vertical limits
            double v_prime = p_prime.dot(v_hat);
            if (v_prime < -projection_plane_h / 2 || v_prime > projection_plane_h / 2) {
                // Point is outside the vertical FOV
                continue;
            }

            // Point is within the FOV
            directions.push_back(direction.normalized());

            if (visualization){
                MoveIt2API_node_->visual_tools->publishLine(starting_pose.translation(), target, rviz_visual_tools::PURPLE, rviz_visual_tools::XXXXSMALL);
            }
        }

        if (visualization){
            MoveIt2API_node_->visual_tools->trigger();
        }

        return directions;
    }



    /**
     * @brief Performs ray casting to find the closest occupied voxel to the starting point with attention to specific target points.
     * 
     * This function performs ray casting to find the closest occupied voxel to the starting point with attention to specific target points.
     * It generates ray end points at a large-enough distance from the starting point to cover the area of the semantic objects, and then casts rays
     * to find the closest occupied voxel to the starting point. This approach ensures that each ray is cast within the FOV from the starting point 
     * to a enough-distant ending point. The process is speed-up using OMP
     * 
     * @param pose The starting pose for ray casting, represented as an Eigen::Isometry3d.
     * @param fov_w The field of view width in degrees.
     * @param fov_h The field of view height in degrees.
     * @return A set of octomap::OcTreeKey keys representing the occupied voxels found by ray casting.
     */
    octomap::KeySet ActiveVisionNbvPlanningPipeline::performRayCasting(Eigen::Isometry3d pose, double fov_w, double fov_h){

        octomap::KeySet finalSet;
        double ray_depth = maxRayDepth_;
        std::vector<Eigen::Vector3d> directions = generateFrustumBaseDirections(pose, fov_w, fov_h, extended_octomap_node_->getRes()*rayStepProportion_);
        RCLCPP_DEBUG(this->get_logger(), "Number of directions: %zu", directions.size());

        octomap::point3d starting_3d_point(pose.translation().x(), pose.translation().y(), pose.translation().z());

        #pragma omp parallel
        {
            octomap::KeySet localSet; // Thread-local set to collect keys
            #pragma omp for nowait // Distribute loop iterations across threads, without waiting for all threads at the end of the loop
            for (size_t i = 0; i < directions.size(); ++i) {
                auto& dir = directions[i];
                Eigen::Vector3d normalized_dir = dir.normalized();
                octomap::point3d direction(normalized_dir.x(), normalized_dir.y(), normalized_dir.z());
                octomap::point3d hit_point;
                if (octree_->castRay(starting_3d_point, direction, hit_point, true, ray_depth)) {
                    octomap::OcTreeKey hit_key = octree_->coordToKey(hit_point);
                    localSet.insert(hit_key);
                }
            }
            #pragma omp critical
            finalSet.insert(localSet.begin(), localSet.end()); // Merge thread-local sets into the final set
        }

        return finalSet;
    }


    /**
     * @brief Performs ray casting with attention to specific target points within the field of view (FOV).
     * 
     * This function performs ray casting based on the semantic area, generating directions toward the starting point and ensuring that each ray
     * is cast within the FOV, effectively covering only the area of the semantic objects. The process is parallelized using OpenMP for efficiency.
     * 
     * @param pose The starting pose for ray casting, represented as an Eigen::Isometry3d.
     * @param fov_w The field of view width in degrees.
     * @param fov_h The field of view height in degrees.
     * @param visualization A boolean flag to enable or disable visualization in RViz.
     * @return An octomap::KeySet containing the keys of the hit points.
     */
    octomap::KeySet ActiveVisionNbvPlanningPipeline::performRayCastingAttention(Eigen::Isometry3d pose, double fov_w, double fov_h, bool visualization=false){

        octomap::KeySet finalSet;
        double ray_depth = maxRayDepth_;
        std::vector<Eigen::Vector3d> directions = generateFrustumBaseAttentionDirections(pose, fov_w, fov_h, visualization);
        RCLCPP_DEBUG(this->get_logger(), "Number of directions: %zu", directions.size());

        octomap::point3d starting_3d_point(pose.translation().x(), pose.translation().y(), pose.translation().z());

        #pragma omp parallel
        {
            octomap::KeySet localSet; // Thread-local set to collect keys
            #pragma omp for nowait // Distribute loop iterations across threads, without waiting for all threads at the end of the loop
            for (size_t i = 0; i < directions.size(); ++i) {
                auto& dir = directions[i];
                Eigen::Vector3d normalized_dir = dir.normalized();
                octomap::point3d direction(normalized_dir.x(), normalized_dir.y(), normalized_dir.z());
                octomap::point3d hit_point;
                if (octree_->castRay(starting_3d_point, direction, hit_point, true, ray_depth)) {
                    octomap::OcTreeKey hit_key = octree_->coordToKey(hit_point);
                    localSet.insert(hit_key);
                }
            }
            #pragma omp critical
            finalSet.insert(localSet.begin(), localSet.end()); // Merge thread-local sets into the final set
        }

        return finalSet;
    }



    /**
     * @brief Calculates the utility of a set of voxels based on a specified utility type.
     * 
     * This function calculates the utility of a set of voxels based on a specified utility type. The utility is calculated based on the confidence
     * of each voxel, and the type of utility calculation is determined by the utility_type parameter. The function returns the total utility value.
     * 
     * @param voxels A set of octomap::OcTreeKey keys representing the voxels.
     * @param utility_type The type of utility calculation to perform.
     * @return The total utility value as a float.
     */
    float ActiveVisionNbvPlanningPipeline::utilityCalculation(octomap::KeySet voxels, std::string utility_type){

        float utility = 0.0;

        if (utility_type == "expected_semantic_information_gain"){
            for (auto key: voxels){
                float confidence = (*extendedOctomapMap_)[key].getConfidence();
                float key_utility = (- confidence * log2(confidence)) - ((1 - confidence) * log2(1 - confidence));
                utility = utility + key_utility;
            }
        }

        // Case when the utility of the viewpoint need to be the sum of the confidences of the voxel
        // This approach is oriented at choosing the viewpoint that have the smallest confidence
        // The final utility will be the highest for the viewpoint with the smallest confidence,
        // the smallest for the one with the greatest confidence (since the greatest value will be chosen it is a negative sum)
        if (utility_type == "minimum_confidence"){
            for (auto key: voxels){
                float confidence = (*extendedOctomapMap_)[key].getConfidence();
                utility = utility - confidence;
            }
        }

        return utility;

    }



    /**
     * @brief Chooses the next-best-view (NBV) pose from a given list of poses based on utility calculation.
     * 
     * This function selects the next-best-view (NBV) pose from a provided vector of poses by calculating the utility of each pose.
     * It performs ray casting to determine the visibility of semantic objects and calculates the utility based on the specified utility type.
     * The pose with the highest utility is selected as the NBV. If no better pose is found, it returns the current pose or a random pose.
     * 
     * Key steps include:
     * 1. **Check Input Vector**: Ensure the input vector of poses is not empty.
     * 2. **Initialize Variables**: Initialize variables for storing pose utilities, maximum utility, and the best pose.
     * 3. **Calculate FOV**: Calculate the horizontal and vertical field of view (FOV) for the viewpoint frustum.
     * 4. **Calculate Scene Bounds**: If central attention is needed, calculate the bounds of the scene and adjust them with specified ratios.
     * 5. **Loop Through Poses**: For each candidate viewpoint pose:
     *    - Visualize the frustum if visualization is enabled.
     *    - Perform ray casting to determine the visibility of semantic objects.
     *    - Calculate the utility of the pose based on the specified utility type.
     *    - Visualize the result of the ray casting and utility calculation if visualization is enabled.
     *    - Update the maximum utility and best pose if the current pose has a higher utility.
     * 6. **Return Best Pose**: Return the pose with the highest utility. If no better pose is found, return the current pose or a random pose.
     * 
     * @param poses A vector of Eigen::Isometry3d poses to choose from.
     * @param current_pose The current pose represented as an Eigen::Isometry3d.
     * @return The next-best-view (NBV) pose as an Eigen::Isometry3d.
     * @throws std::runtime_error if the input vector of poses is empty.
     */
    Eigen::Isometry3d ActiveVisionNbvPlanningPipeline::chooseNBV(const std::vector<Eigen::Isometry3d>& poses, Eigen::Isometry3d current_pose) {
        // Check if the input vector is empty
        if (poses.empty()) {
            throw std::runtime_error("Input vector of poses is empty."); // Throw an exception if there are no poses to choose from
        }

        std::vector<float> poseUtilities;
        maxUtility_ = 0.0;
        Eigen::Isometry3d bestPose;
        bool isMaxUtilityUpdated = false;



        // Calculate horizontal and vertical FOV for the viewpoint frustum
        double fx = current_camera_info_msg_->k[0];
        double fy = current_camera_info_msg_->k[4];
        int w = current_camera_info_msg_->width;
        int h = current_camera_info_msg_->height;

        double fov_w, fov_h;
        if (hardware_protocol_ == "ignition"){
            fov_w = 2.0 * std::atan(static_cast<double>(w) / (4.0 * fx));
            fov_h = 2.0 * std::atan(static_cast<double>(h) / (4.0 * fy));
        }
        else {
            fov_w = 2.0 * std::atan(static_cast<double>(w) / (2.0 * fx));
            fov_h = 2.0 * std::atan(static_cast<double>(h) / (2.0 * fy));
        }
        fov_w = fov_w * (180.0 / M_PI); // in degrees
        fov_h = fov_h * (180.0 / M_PI); // in degrees



        // If central attention is needed, calculate bounds of the scene
        // Calculate the center and bounds of the scene and correct them with a specific ratio
        if (rayCastingType_ == "full_attention" || rayCastingType_ == "full_attention_distance"){
            furthestPoint_ = findFurthestPoint(initialPositionCartesian_);
            closestPoint_ = findClosestPoint(initialPositionCartesian_);
            highestPoint_ = findHighestPoint(initialPositionCartesian_);
            lowestPoint_ = findLowestPoint(initialPositionCartesian_);
            leftMostPoint_ = findLeftMostPoint(initialPositionCartesian_);
            rightMostPoint_ = findRightMostPoint(initialPositionCartesian_);

            // Adjust the bounds by the specified ratios
            auto adjustBound = [](double min, double max, double ratio) -> std::pair<double, double> {
                double range = max - min;
                double adjustment = range * (1 - 1.0 / ratio) / 2.0; // Adjust both sides equally
                return {min + adjustment, max - adjustment};
            };

            std::tie(newClosest_, newFurthest_) = adjustBound(closestPoint_.x(), furthestPoint_.x(), centralAttentionFrontDistanceRatio_);
            std::tie(newLowest_, newHighest_) = adjustBound(lowestPoint_.z(), highestPoint_.z(), centralAttentionHeightDistanceRatio_);
            std::tie(newLeftMost_, newRightMost_) = adjustBound(leftMostPoint_.y(), rightMostPoint_.y(), centralAttentionWidthDistanceRatio_);
        }

        // Save sum of calculation time for each candidate viewpoint
        double elapsed_times = 0;
        auto total_start = std::chrono::high_resolution_clock::now();

        // Start a loop for each valid candidate viewpoint
        for (auto& pose : poses){
            auto start = std::chrono::high_resolution_clock::now();

            if (rayCastingVis_){
                // Visualize frustum
                visualizeFrustum(pose, fov_w, fov_h, 1.0);
                visualizeFrustumBase(pose, fov_w, fov_h, 1.0);
                visualizeArrowPose(pose, 0.2, rviz_visual_tools::YELLOW, rviz_visual_tools::LARGE);
                MoveIt2API_node_->visual_tools->trigger();
            }



            // Ray casting
            RCLCPP_DEBUG(this->get_logger(), "Performing %s ray casting...", rayCastingType_.c_str());
            octomap::KeySet rayCastingVoxels;

            if (rayCastingType_ == "naive"){
                rayCastingVoxels = performNaiveRayCasting(pose, fov_w, fov_h, rayCastingVis_);
            }
            else if (rayCastingType_ == "normal"){
                rayCastingVoxels = performRayCasting(pose, fov_w, fov_h);
            }
            else {
                rayCastingVoxels = performRayCastingAttention(pose, fov_w, fov_h, rayCastingVis_);
            }
            
            RCLCPP_DEBUG(this->get_logger(), "Ray casting performed.");

            if (rayCastingVis_){
                // Visualize points related to the voxel obtained by attention ray casting
                for (auto key: rayCastingVoxels){
                    octomap::point3d centerPoint = octree_->keyToCoord(key);
                    Eigen::Vector3d centerVector(centerPoint.x(), centerPoint.y(), centerPoint.z());
                    MoveIt2API_node_->visual_tools->publishSphere(centerVector, rviz_visual_tools::YELLOW, rviz_visual_tools::MEDIUM);
                }
                RCLCPP_DEBUG(this->get_logger(), "Visualizing result of the ray casting...");
                MoveIt2API_node_->visual_tools->trigger();
                
                rclcpp::sleep_for(std::chrono::milliseconds(2000));
                MoveIt2API_node_->visual_tools->deleteAllMarkers();
            }



            // Utility calculation
            RCLCPP_DEBUG(this->get_logger(), "Calculating %s for the current pose...", utilityType_.c_str());
            octomap::KeySet targetVoxels;
            for (auto key: rayCastingVoxels){
                if ((*extendedOctomapMap_)[key].getSemanticClass() == prompt_){
                    targetVoxels.insert(key);
                }
            }

            float poseUtility = utilityCalculation(targetVoxels, utilityType_);
            

            if (utilityVis_){
                // Visualize points related to the voxel obtained by attention ray casting
                for (auto key: targetVoxels){
                    octomap::point3d centerPoint = octree_->keyToCoord(key);
                    Eigen::Vector3d centerVector(centerPoint.x(), centerPoint.y(), centerPoint.z());
                    MoveIt2API_node_->visual_tools->publishSphere(centerVector, rviz_visual_tools::YELLOW, rviz_visual_tools::MEDIUM);
                }
                RCLCPP_DEBUG(this->get_logger(), "Visualizing voxels used for future entropy calculation...");
                MoveIt2API_node_->visual_tools->trigger();

                RCLCPP_DEBUG(this->get_logger(), "The %s of this pose is %f", utilityType_.c_str(), poseUtility);
                for (auto key: targetVoxels){
                    RCLCPP_DEBUG(this->get_logger(), "The confidence of the voxel is %f", (*extendedOctomapMap_)[key].getConfidence());
                }
                
                rclcpp::sleep_for(std::chrono::milliseconds(2000));
                MoveIt2API_node_->visual_tools->deleteAllMarkers();
            }

            RCLCPP_DEBUG(this->get_logger(), "Utility calculated.");



            // Total utility calculation
            if (rayCastingType_ == "full_attention_distance"){
                double distance = (pose.translation() - current_pose.translation()).norm();
                poseUtility = poseUtility * exp(-distance);
            }
            poseUtilities.push_back(poseUtility);

            RCLCPP_DEBUG(this->get_logger(), 
                "The current pose %f, %f, %f pose has total utility is %f", 
                pose.translation().x(), pose.translation().y(), pose.translation().z(), poseUtility);



            // Check if the current pose's total utility is the highest
            if (poseUtility > maxUtility_) {
                maxUtility_ = poseUtility;
                bestPose = pose;
                isMaxUtilityUpdated = true;
                semanticFound_ = true;
            }

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            elapsed_times = elapsed_times + elapsed.count();
            RCLCPP_DEBUG(this->get_logger(), "Time taken for the valid candidate viewpoint: %.6f seconds", elapsed.count());

        }
        auto total_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> total_elapsed = total_end - total_start;

        // Print mean duration of candidate viewpoint operation and total
        RCLCPP_INFO(this->get_logger(), "Mean time taken for the valid candidate viewpoint: %.6f seconds", elapsed_times / poses.size());
        RCLCPP_INFO(this->get_logger(), "Total duration of ray casting for all valid candidate viewpoints (%zu): %.6f seconds", poses.size(), total_elapsed.count());


        // Check if maxUtility was updated
        if (!isMaxUtilityUpdated) {
            // If during the entire planning some semantic has been found, return the current pose
            if (semanticFound_){
                RCLCPP_INFO(this->get_logger(), "No better pose found. Returning current pose %f, %f, %f", current_pose.translation().x(), current_pose.translation().y(), current_pose.translation().z());
                return current_pose;
            }
            // If during the entire planning no semantic is found yet, return a random pose
            else {
                RCLCPP_INFO(this->get_logger(), "No better pose found. Returning random pose");
                Eigen::Isometry3d random_pose = chooseNBVRandom(poses);
                return random_pose;
            }
        }
        else{
            RCLCPP_INFO(this->get_logger(), 
                "The NBV pose %f, %f, %f has utility %f", bestPose.translation().x(), bestPose.translation().y(), bestPose.translation().z(), maxUtility_);
            return bestPose;
        }

    }


    /**
     * @brief Calculates the reconstruction metric by comparing the ground truth octomap with the reconstructed octomap.
     * 
     * This function calculates the reconstruction metric by comparing the ground truth octomap with the reconstructed octomap.
     * It identifies true positives, false positives, and false negatives, and computes the F1 score based on these values.
     * Optionally, it visualizes the points in RViz.
     * 
     * Key steps include:
     * 1. **Initialize Point Vectors**: Initialize vectors to store points from the ground truth and reconstructed octomaps.
     * 2. **Fill Ground Truth Points**: Iterate through the ground truth octomap and fill the vector with points.
     * 3. **Visualize Ground Truth Points (Optional)**: Optionally, visualize the ground truth points in RViz.
     * 4. **Fill Reconstruction Points**: Iterate through the reconstructed octomap and fill the vector with points.
     * 5. **Visualize Reconstruction Points (Optional)**: Optionally, visualize the reconstructed points in RViz.
     * 6. **Calculate TP, FP, FN**: Identify true positives, false positives, and false negatives by comparing the two sets of points.
     * 7. **Visualize TP, FP, FN (Optional)**: Optionally, visualize the true positives, false positives, and false negatives in RViz.
     * 8. **Compute F1 Score**: Calculate the precision, recall, and F1 score based on the TP, FP, and FN values.
     * 9. **Return F1 Score**: Return the computed F1 score.
     * 
     * @param visualization A boolean flag to enable or disable visualization in RViz.
     * @return The F1 score representing the reconstruction metric.
     */
    double ActiveVisionNbvPlanningPipeline::reconstructionMetric(bool visualization){
        
        MoveIt2API_node_->visual_tools->deleteAllMarkers();
        MoveIt2API_node_->visual_tools->setBaseFrame(this->base_frame_id_);
        MoveIt2API_node_->visual_tools->trigger();

        // Save set of point referring to the truth and to the reconstruction
        std::vector<Eigen::Vector3d> truth_points;
        std::vector<Eigen::Vector3d> reconstruction_points;


        
        // Fill the octree truth points vector
        for (auto it = octree_truth_->begin(octree_truth_->getResolution()), end = octree_truth_->end(); it != end; ++it) {
            double x = it.getX();
            double y = it.getY();
            double z = it.getZ();
            Eigen::Vector3d point_eigen(x, y, z);
            truth_points.push_back(point_eigen);
            if (visualization){
                MoveIt2API_node_->visual_tools->publishSphere(point_eigen, rviz_visual_tools::GREEN, rviz_visual_tools::SMALL);
            }
        }
        if (visualization){
            RCLCPP_INFO(this->get_logger(), "Visualizing octree truth...");
            MoveIt2API_node_->visual_tools->publishText(textPose_, "GroundTruth", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
            MoveIt2API_node_->visual_tools->trigger();
            rclcpp::sleep_for(std::chrono::milliseconds(5000));
        }



        // Fill the reconstruction points vector
        for (auto it = extendedOctomapMap_->begin(); it != extendedOctomapMap_->end(); ++it) {
            const octomap::OcTreeKey& key = it->first;
            ExtendedOctomapData& data = it->second;

            // Check if the semantic class matches the target
            if (data.getSemanticClass() == prompt_) {
                // Retrieve the 3D position using the octree
                octomap::point3d point = octree_->keyToCoord(key);
                Eigen::Vector3d point_eigen(point.x(), point.y(), point.z());    
                reconstruction_points.push_back(point_eigen);   
                if (visualization){                 
                    MoveIt2API_node_->visual_tools->publishSphere(point_eigen, rviz_visual_tools::BLUE, rviz_visual_tools::SMALL);
                }
            }
        }
        if (visualization){
            RCLCPP_INFO(this->get_logger(), "Visualizing reconstruction octree...");
            MoveIt2API_node_->visual_tools->publishText(textPose_, "Reconstruction", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
            MoveIt2API_node_->visual_tools->trigger();
            rclcpp::sleep_for(std::chrono::milliseconds(5000));
            MoveIt2API_node_->visual_tools->deleteAllMarkers();
            MoveIt2API_node_->visual_tools->setBaseFrame(this->base_frame_id_);
            MoveIt2API_node_->visual_tools->trigger();
        }



        // Calculate TP, FP, FN
        std::vector<Eigen::Vector3d> true_positives;
        std::vector<Eigen::Vector3d> false_positives;
        std::vector<Eigen::Vector3d> false_negatives;

        // Sort both vectors for binary search
        auto pointComparator = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
            return std::tie(a.x(), a.y(), a.z()) < std::tie(b.x(), b.y(), b.z());
        };

        std::sort(truth_points.begin(), truth_points.end(), pointComparator);
        std::sort(reconstruction_points.begin(), reconstruction_points.end(), pointComparator);

    
        // Custom comparison function to compare points within a tolerance
        auto pointComparatorWithTolerance = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
            return (a - b).norm() < 1e-6;
        };


        // Find True Positives and False Negatives
        for (const auto& point : truth_points) {
            if (std::any_of(reconstruction_points.begin(), reconstruction_points.end(),
                            [&](const Eigen::Vector3d& recon_point) {
                                return pointComparatorWithTolerance(point, recon_point);
                            })) {
                true_positives.push_back(point);
            } else {
                false_negatives.push_back(point);
            }
        }

        // Find False Positives
        for (const auto& point : reconstruction_points) {
            if (!std::any_of(truth_points.begin(), truth_points.end(),
                            [&](const Eigen::Vector3d& truth_point) {
                                return pointComparatorWithTolerance(point, truth_point);
                            })) {
                false_positives.push_back(point);
            }
        }


        // Visualize TP, FP, FN
        if (visualization){
            RCLCPP_INFO(this->get_logger(), "Visualizing TP, FP, FN...");
            for (auto point: true_positives){
                MoveIt2API_node_->visual_tools->publishSphere(point, rviz_visual_tools::GREEN, rviz_visual_tools::SMALL);
            }
            MoveIt2API_node_->visual_tools->publishText(textPose_, "TruePositive", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
            MoveIt2API_node_->visual_tools->trigger();
            rclcpp::sleep_for(std::chrono::milliseconds(5000));
            MoveIt2API_node_->visual_tools->deleteAllMarkers();
            MoveIt2API_node_->visual_tools->setBaseFrame(this->base_frame_id_);
            MoveIt2API_node_->visual_tools->trigger();
            for (auto point: false_negatives){
                MoveIt2API_node_->visual_tools->publishSphere(point, rviz_visual_tools::YELLOW, rviz_visual_tools::SMALL);
            }
            MoveIt2API_node_->visual_tools->publishText(textPose_, "FalseNegative", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
            MoveIt2API_node_->visual_tools->trigger();
            rclcpp::sleep_for(std::chrono::milliseconds(5000));
            MoveIt2API_node_->visual_tools->deleteAllMarkers();
            MoveIt2API_node_->visual_tools->setBaseFrame(this->base_frame_id_);
            MoveIt2API_node_->visual_tools->trigger();
            for (auto point: false_positives){
                MoveIt2API_node_->visual_tools->publishSphere(point, rviz_visual_tools::RED, rviz_visual_tools::SMALL);
            }
            MoveIt2API_node_->visual_tools->publishText(textPose_, "FalsePositive", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
            MoveIt2API_node_->visual_tools->trigger();
            rclcpp::sleep_for(std::chrono::milliseconds(5000));
            MoveIt2API_node_->visual_tools->deleteAllMarkers();
            MoveIt2API_node_->visual_tools->setBaseFrame(this->base_frame_id_);
            MoveIt2API_node_->visual_tools->trigger();
        }


        // Compute F1 score
        auto TP = true_positives.size();
        auto FN = false_negatives.size();
        auto FP = false_positives.size();

        double precision = TP + FP == 0 ? 0 : static_cast<double>(TP) / (TP + FP);
        double recall = TP + FN == 0 ? 0 : static_cast<double>(TP) / (TP + FN);
        double f1 = precision + recall == 0 ? 0 : 2 * (precision * recall) / (precision + recall);

        return f1;
    }


}
