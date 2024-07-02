#include <fruit_picking_planning/active_vision_nbv_planning_pipeline.hpp>
#include <random>


namespace active_vision_nbv_planning_pipeline{
    
    // Constructor
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


        // Initialize the maximum number of steps for the NBV planning
        maxNBVPlanningSteps_ = 8;   

        
        
    }






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
        candidateViewpoints_= generatePlaneCandidateViewpoints(
            initialPositionCartesian_, orientations_, planeTypeCandidateViewpoints_, candidateViewpointsNumber_, movementRange_);
        NBV_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
        int total_candidates = candidateViewpointsNumber_ + candidateViewpointsNumber_ * orientations_;
        RCLCPP_INFO(this->get_logger(), "Initial position and %d candidate viewpoints in a %s shape with movement range of %.2f mt created.", total_candidates, planeTypeCandidateViewpoints_.c_str(), movementRange_);        




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

        bool valid_motion = MoveIt2API_node_->robotPlanAndMove(
            eigenIsometry3dToPoseStamped(initialPositionCartesian_), 
            "initial_position",
            false);
        if (!valid_motion) {
			RCLCPP_ERROR(this->get_logger(), "Could not move to initial position");
			return;
		}
        RCLCPP_INFO(this->get_logger(), "Initial position reached.");



	    // // Alternative way: with joint space goal
        // bool valid_motion = MoveIt2API_node_->robotPlanAndMove(initialPosition_, "initial_position");
        // if (!valid_motion) {
		// 	RCLCPP_ERROR(this->get_logger(), "Could not move to initial position");
		// 	return;
		// }
        // RCLCPP_INFO(this->get_logger(), "Initial position reached.");


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




            // // Create segmentation request for the server

            // auto request = std::make_shared<fruit_picking_interfaces::srv::YOLOWorldSegmentation::Request>();
            // request->image = *working_rgb_msg;
            // request->text_prompt = this->prompt_;
            // request->confidence_threshold = this->confidence_threshold_;
            // request->nms_threshold = this->nms_confidence_threshold_;




            // // Wait for the server to be active

            // while (!this->client_->wait_for_service(std::chrono::seconds(1))) {
            //     if (!rclcpp::ok()) {
            //         RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            //         return;
            //     }
            //     RCLCPP_WARN(this->get_logger(), "Service not available, waiting again...");
            // }

            // auto result = client_->async_send_request(request);




            // // Wait for the response, and save them once received

            // if (rclcpp::spin_until_future_complete(client_node_, result) ==
            //     rclcpp::FutureReturnCode::SUCCESS)
            // {
            //     auto response = result.get();
            //     masks_images_array_ = std::make_shared<const ImageArray>(response->masks_images_array);
            //     merged_masks_image_ = std::make_shared<const Image>(response->merged_masks_images);
            //     confidences_ = std::make_shared<const Confidence>(response->confidences);
            //     RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
            //     RCLCPP_INFO(this->get_logger(), "Segmentation response obtained.");
            // }


            // // Publish segmented image
            // segmentedImagePub_->publish(*merged_masks_image_);



            // // Create full and segmented pointcloud
            // if (usePartialPointcloud_){
            //     RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
            //     RCLCPP_INFO(this->get_logger(), "Creating partial pointcloud...");
            //     partialPointcloud_ = segmented_pointcloud_node_->imageCb(
            //         working_depth_msg, 
            //         merged_masks_image_, 
            //         working_camera_info_msg);
            // } 
            // else {
            //     RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
            //     RCLCPP_INFO(this->get_logger(), "Creating full pointcloud...");
            //     fullPointcloud_ = pointcloud_node_->imageCb(
            //         working_depth_msg, 
            //         working_rgb_msg, 
            //         working_camera_info_msg);
            // }
            // RCLCPP_INFO(this->get_logger(), "Creating segmented pointclouds array...");
            
            // segmentedPointcloudArray_ = segmented_pointcloud_node_->imageArrayCb(
            //     working_depth_msg,
            //     masks_images_array_,
            //     working_camera_info_msg,
            //     confidences_);
            // RCLCPP_INFO(this->get_logger(), "Pointclouds created.");



            // // Publish full and segmented pointclouds for visualization
            // segmentedPointcloud_ = segmented_pointcloud_node_->imageCb(
            //     working_depth_msg, 
            //     merged_masks_image_, 
            //     working_camera_info_msg);
            // segmentedPointcloudPub_->publish(*segmentedPointcloud_);

            


            // // Update octomap and publish visualization
            // RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
            // RCLCPP_INFO(this->get_logger(), "Updating octomap...");
            // if (usePartialPointcloud_){
            //     extended_octomap_node_->insertPartialCloudCallback(partialPointcloud_, working_tf);
            //     extended_octomap_node_->insertSegmentedPointcloudsArrayCallback(segmentedPointcloudArray_, working_tf, partialPointcloud_);
            // }
            // else {
            //     extended_octomap_node_->insertCloudCallback(fullPointcloud_);
            //     extended_octomap_node_->insertSegmentedPointcloudsArrayCallback(segmentedPointcloudArray_, working_tf, fullPointcloud_);
            // }
            //
            // extendedOctomapMap_ = extended_octomap_node_->getExtendedOctomapMap();
            //
            // RCLCPP_INFO(this->get_logger(), "Octomap updated.");




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
            // NBV_pose_ = chooseNBVRandom(validCandidateViewpoints_);
            NBV_pose_ = chooseNBV(validCandidateViewpoints_);
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
            MoveIt2API_node_->visual_tools->publishText(findUpperCenterPose(validCandidateViewpoints_), "NBV_pose", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
            MoveIt2API_node_->visual_tools->trigger();


            rclcpp::sleep_for(std::chrono::milliseconds(1500));

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
            rclcpp::sleep_for(std::chrono::milliseconds(1500));





            RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
            RCLCPP_WARN(this->get_logger(), "NBV planning pipeline step terminated.");


        }

        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
        RCLCPP_WARN(this->get_logger(), "NBV planning pipeline terminated.");


    }



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



    void ActiveVisionNbvPlanningPipeline::saveData(
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
        // Used to tell the main function that is time to execute
        data_received_ = true;
        // Used to block the wait, and to check the value of the above flag
        data_cond_.notify_one();

        RCLCPP_DEBUG(this->get_logger(), "Internal data updated.");

    }



    std::array<double, 6> ActiveVisionNbvPlanningPipeline::getInitialPosition(){

        // Load yaml configuration files
        std::string package_path = ament_index_cpp::get_package_share_directory("fruit_picking_planning");
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

        auto addPoseWithOrientations = [&](Eigen::Isometry3d newPose) {
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

        if (planeTypeCandidateViewpoints_ == "circle") {
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

                    addPoseWithOrientations(newPose);
                    totalPosesPlaced++;
                    if (totalPosesPlaced >= N) break;
                }
                if (totalPosesPlaced >= N) break;
            }
        } else {
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

                    addPoseWithOrientations(newPose);
                }
            }
        }

        return poses;
    }



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



    void ActiveVisionNbvPlanningPipeline::visualizeFrustum(const Eigen::Isometry3d& starting_pose, double fov_w_deg, double fov_h_deg) {
        // Convert FOV from degrees to radians
        double fov_w_rad = fov_w_deg * (M_PI / 180.0);
        double fov_h_rad = fov_h_deg * (M_PI / 180.0);

        // Calculate half-angles for simplicity
        double half_fov_w = fov_w_rad / 2.0;
        double half_fov_h = fov_h_rad / 2.0;

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

        // Starting point for the lines is the apex of the frustum
        Eigen::Vector3d starting_point = starting_pose.translation();

        // Visualize the frustum sides
        for (const auto& dir : directions) {
            // Transform direction vector to world frame and scale to desired length (e.g., 1 meter)
            Eigen::Vector3d end_point = starting_point + (starting_pose.rotation() * dir) * 1.0;
            MoveIt2API_node_->visual_tools->publishLine(starting_point, end_point, rviz_visual_tools::YELLOW, rviz_visual_tools::XSMALL);
        }

        MoveIt2API_node_->visual_tools->trigger();
    }



    void ActiveVisionNbvPlanningPipeline::visualizeFrustumBase(const Eigen::Isometry3d& starting_pose, double fov_w_deg, double fov_h_deg) {
        // Convert FOV from degrees to radians
        double fov_h_rad = fov_h_deg * (M_PI / 180.0);
        double fov_w_rad = fov_w_deg * (M_PI / 180.0);

        // Calculate half-angles for simplicity
        double half_fov_h = fov_h_rad / 2.0;
        double half_fov_w = fov_w_rad / 2.0;

        // Depth is set to 1 meter
        double depth = 1.0;

        // Calculate the offsets at the depth
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



    Eigen::Isometry3d ActiveVisionNbvPlanningPipeline::chooseNBV(const std::vector<Eigen::Isometry3d>& poses) {
        // Check if the input vector is empty
        if (poses.empty()) {
            throw std::runtime_error("Input vector of poses is empty."); // Throw an exception if there are no poses to choose from
        }

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
        fov_w = fov_w * (180.0 / M_PI);
        fov_h = fov_h * (180.0 / M_PI);


        // Visualize frustum
        for (auto pose : poses){
            visualizeFrustum(pose, fov_w, fov_h);
            visualizeFrustumBase(pose, fov_w, fov_h);
            visualizeArrowPose(pose, 0.2, rviz_visual_tools::YELLOW, rviz_visual_tools::LARGE);
            MoveIt2API_node_->visual_tools->trigger();

            rclcpp::sleep_for(std::chrono::milliseconds(2000));
            MoveIt2API_node_->visual_tools->deleteAllMarkers();
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


}