#include <fruit_picking_planning/active_vision_predefined_planning_pipeline.hpp>

namespace active_vision_predefined_planning_pipeline{
    
    // Constructor
    ActiveVisionPredefinedPlanningPipeline::ActiveVisionPredefinedPlanningPipeline(
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
        RCLCPP_INFO(this->get_logger(), "Predefined planning pipeline constructor started.");


        // Read arguments and save the node into some variables
        MoveIt2API_node_ = MoveIt2API_creator;
        pointcloud_node_ = pointcloud_creator;
        segmented_pointcloud_node_ = segmented_pointcloud_creator;
        extended_octomap_node_ = extended_octomap_creator;
        client_node_ = segmentationClientNode;

        // Read parameters
        frame_id_ = this->declare_parameter("frame_id", "world");
        base_frame_id_ = this->declare_parameter("base_frame_id", "igus_rebel_base_link");
        queue_size_ = this->declare_parameter<int>("queue_size", 5);
        prompt_ = this->declare_parameter("segmentation_prompt", "tomato");
        confidence_threshold_ = this->declare_parameter<float>("confidence_threshold", 0.001);
        nms_confidence_threshold_ = this->declare_parameter<float>("nms_threshold", 0.2);
        usePartialPointcloud_ = this->declare_parameter("partial_pointcloud_subscription", true); 
        predefinedPlanning_ = this->declare_parameter("predefined_planning", "zig_zag_planning_wide");

        RCLCPP_INFO(this->get_logger(), "Parameters and arguments initialized..");



        // Initialize segmentation service
        this->client_ = client_node_->create_client<fruit_picking_interfaces::srv::YOLOWorldSegmentation>("/yolo_world_service");
        RCLCPP_INFO(this->get_logger(), "Segmentation client initialized..");


        // Initialize segmented image visualization publisher
        segmentedImagePub_ = create_publisher<Image>("/visualization/yolo_world_segmented_image", rclcpp::SensorDataQoS(rclcpp::KeepLast(3)));

        // Initialize full and segmented pointcloud visualization publisherrclcpp::SensorDataQoS()
        segmentedPointcloudPub_ = create_publisher<PointCloud2>("/visualization/segmented_pointcloud", rclcpp::SensorDataQoS(rclcpp::KeepLast(3)));

        // Initialize extended octomap visualization publishers
        extended_octomap_node_->createVisualizations();

        RCLCPP_INFO(this->get_logger(), "Segmented and octomap visual tools initialized.");        



        // Initialize Moveit2 variables
        PlanningPoses_ = createPlanningPoses();
        CartesianPlanningPoses_= std::vector<Eigen::Isometry3d>(PlanningPoses_.size());
        RCLCPP_INFO(this->get_logger(), "Initial position and %s positions created.", predefinedPlanning_.c_str());        

        
    }


    void ActiveVisionPredefinedPlanningPipeline::createDataSub(){
        try {
            data_sync_ = std::make_shared<DataSynchronizer>(DataSyncPolicy(queue_size_), sub_rgb_, sub_depth_, sub_camera_info_);
            data_sync_->registerCallback(
                std::bind(
                    &ActiveVisionPredefinedPlanningPipeline::saveData,
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



    void ActiveVisionPredefinedPlanningPipeline::saveData(
        const Image::ConstSharedPtr & rgb_msg,
        const Image::ConstSharedPtr & depth_msg,
        const CameraInfo::ConstSharedPtr & camera_info_msg)
    {
        // Lock the variables till the function terminates for concurrency purposes
        std::lock_guard<std::mutex> lock(data_mutex_);

        RCLCPP_DEBUG(this->get_logger(), "Saving data...");

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

        RCLCPP_DEBUG(this->get_logger(), "Internal data updated.");
        // From this moment on, when the function terminates (and the scope is destroyed), the lock is released
        // But if no other thread get the lock, again this function will get it one more time since it is called continuosly 
        // by the sync
        // Moreover, the main function can not get the lock if data received is still false. This assures the fact that
        // this function restarts till all the data are obtained (thanks to the bool flag setted if the tf is received)

    }








    void ActiveVisionPredefinedPlanningPipeline::ActiveVisionPredefinedPlanningPipelineThread(){
        
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Predefined planning pipeline started.");
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
        // set the rate for the main thread
	    rclcpp::Rate rate(15);


    

        // Visualize planning viewpoints
        RCLCPP_INFO(this->get_logger(), "Visualize planning viewpoints..");

        // Create a vector with all the position names
        std::vector<std::string> positions_names(PlanningPoses_.size());
        for (size_t i = 0; i < PlanningPoses_.size(); ++i) {
            if (i == 0){
                positions_names[i] = "initial_position";
            }
            else {
                positions_names[i] = "position_" + std::to_string(i);
            }
        }
        // Set the fixed frame id of the visualization
        MoveIt2API_node_->visual_tools->setBaseFrame(this->base_frame_id_);
        // For each pose, get the cartesian pose and publish a marker on it
        for (size_t i = 0; i < PlanningPoses_.size(); ++i){
            CartesianPlanningPoses_[i] = MoveIt2API_node_->fromJointSpaceGoalToCartesianPose(PlanningPoses_[i]);
            MoveIt2API_node_->visual_tools->publishAxisLabeled(CartesianPlanningPoses_[i], positions_names[i], rviz_visual_tools::MEDIUM, rviz_visual_tools::GREEN);
        }
        MoveIt2API_node_->visual_tools->trigger();




        // Move to the initial position
        RCLCPP_INFO(this->get_logger(), "Moving to initial position..");
	    bool valid_motion = MoveIt2API_node_->robotPlanAndMove(PlanningPoses_[0], "initial_position");
        if (!valid_motion) {
			RCLCPP_ERROR(this->get_logger(), "Could not move to initial position");
			return;
		}
        RCLCPP_INFO(this->get_logger(), "Initial position reached.");



        // Initialize data subscriber
        RCLCPP_INFO(this->get_logger(), "Creating data subscriber...");
        this->createDataSub();
        RCLCPP_INFO(this->get_logger(), "Data subscriber created");



        // Create a loop that will end when there are not new positions to move to
        // Start from the second element because the first is the first position
        for (size_t i = 1; i < PlanningPoses_.size(); ++i) {

            RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
            RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
            RCLCPP_INFO(this->get_logger(), "Predefined planning pipeline step started.");
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

            auto request = std::make_shared<fruit_picking_interfaces::srv::YOLOWorldSegmentation::Request>();
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




            // Move to the next position
            RCLCPP_INFO(this->get_logger(), "Moving to position %zu...", i);
            RCLCPP_INFO(this->get_logger(), "Translation: [%f, %f, %f], Rotation (Quaternion): [%f, %f, %f, %f]", 
                CartesianPlanningPoses_[i].translation().x(), 
                CartesianPlanningPoses_[i].translation().y(), 
                CartesianPlanningPoses_[i].translation().z(), 
                Eigen::Quaterniond(CartesianPlanningPoses_[i].rotation()).x(), 
                Eigen::Quaterniond(CartesianPlanningPoses_[i].rotation()).y(), 
                Eigen::Quaterniond(CartesianPlanningPoses_[i].rotation()).z(), 
                Eigen::Quaterniond(CartesianPlanningPoses_[i].rotation()).w());
            valid_motion = MoveIt2API_node_->robotPlanAndMove(PlanningPoses_[i], positions_names[i]);
            if (!valid_motion) {
                RCLCPP_ERROR(this->get_logger(), "Could not move to position %zu.", i);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Position %zu reached.", i);




            // Sleep to allow the robot to be fully on position
            RCLCPP_INFO(this->get_logger(), "Robot settlement...");
            rclcpp::sleep_for(std::chrono::milliseconds(2000));






            RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
            RCLCPP_WARN(this->get_logger(), "Predefined planning pipeline step terminated.");


        }

        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
        RCLCPP_WARN(this->get_logger(), "Predefined planning pipeline terminated.");


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






    }




    std::vector<std::array<double, 6>> ActiveVisionPredefinedPlanningPipeline::createPlanningPoses(){


        // Load yaml configuration files
        std::string package_path = ament_index_cpp::get_package_share_directory("fruit_picking_planning");
        YAML::Node joint_limits = YAML::LoadFile(package_path + "/config/joint_limits.yaml");
        YAML::Node planning_waypoints = YAML::LoadFile(package_path + "/config/predefined_planning_waypoints.yaml");


        // Check if the file was loaded successfully
        if (joint_limits.IsNull()) {
            throw std::runtime_error("Failed to load joint limits config file.");
        }
        if (planning_waypoints.IsNull()) {
            throw std::runtime_error("Failed to load planning waypoints config file.");
        }


        // Define final vector
        std::vector<std::array<double, 6>> poses;


        // Define the limit variables and convert degrees to radians
        std::array<float, 6> min_deg, max_deg, min_rad, max_rad;
        for (int i = 0; i < 6; ++i) {
            std::string joint_key = "joint_" + std::to_string(i + 1);
            min_deg[i] = joint_limits[joint_key]["min"].as<float>();
            max_deg[i] = joint_limits[joint_key]["max"].as<float>();
            min_rad[i] = min_deg[i] * M_PI / 180.0;
            max_rad[i] = max_deg[i] * M_PI / 180.0;
        }


        // Calculate all the planning position based in the planning_waypoints configuration
        for (auto it = planning_waypoints[predefinedPlanning_].begin(); it != planning_waypoints[predefinedPlanning_].end(); ++it) {
            // Each position now refers to one of the positions in the YAML
            auto future_position = it->second; // Assuming the structure is a map or similar

            std::array<double, 6> future_pose;
            for (int i = 0; i < 6; ++i) {
                std::string joint_key = "joint_" + std::to_string(i + 1);
                float joint_value = future_position[joint_key].as<float>();
                if (joint_value < 0) {
                    future_pose[i] = min_rad[i] * joint_value / min_deg[i];
                } else {
                    future_pose[i] = max_rad[i] * joint_value / max_deg[i];
                }
            }
            poses.push_back(future_pose);
        }

        return poses;
    }


}