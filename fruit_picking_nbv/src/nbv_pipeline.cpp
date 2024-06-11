#include <fruit_picking_nbv/nbv_pipeline.hpp>

namespace nbv_pipeline{
    
    // Constructor
    NBVPipeline::NBVPipeline(
        std::shared_ptr<depth_image_proc::PointCloudXyzrgbNode> pointcloud_creator,
        std::shared_ptr<segmented_pointcloud::SegmentedPointcloud> segmented_pointcloud_creator,
        std::shared_ptr<extended_octomap_server::ExtendedOctomapServer> extended_octomap_creator,
        std::shared_ptr<rclcpp::Node> segmentationClientNode,
        const rclcpp::NodeOptions &options, 
        const std::string node_name): 
        Node(node_name, options)
    {
        RCLCPP_INFO(this->get_logger(), "NBV pipeline constructor started.");

        // Read arguments
        pointcloud_ = pointcloud_creator;
        segmented_pointcloud_ = segmented_pointcloud_creator;
        extended_octomap_ = extended_octomap_creator;
        client_node_ = segmentationClientNode;

        // Read parameters
        frame_id_ = this->declare_parameter("frame_id", "world");
        queue_size_ = this->declare_parameter<int>("queue_size", 5);
        prompt_ = this->declare_parameter("segmentation_prompt", "tomato");
        confidence_threshold_ = this->declare_parameter<float>("confidence_threshold", 0.001);
        nms_confidence_threshold_ = this->declare_parameter<float>("nms_threshold", 0.2);


        // Initialize segmentation service
        this->client_ = client_node_->create_client<fruit_picking_interfaces::srv::YOLOWorldSegmentation>("/yolo_world_service");

    }



    void NBVPipeline::createDataSub(){
        try {
            data_sync_ = std::make_shared<DataSynchronizer>(DataSyncPolicy(queue_size_), sub_rgb_, sub_depth_, sub_camera_info_);
            data_sync_->registerCallback(
                std::bind(
                    &NBVPipeline::saveData,
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
            RCLCPP_ERROR(this->get_logger(), "[NBV][createDataSub] %s",e.what());
        } catch (...) {
            // This will catch all other exceptions
            RCLCPP_ERROR(this->get_logger(), "[NBV][createDataSub] Generic error.");
        }        
    }



    void NBVPipeline::saveData(
        const Image::ConstSharedPtr & rgb_msg,
        const Image::ConstSharedPtr & depth_msg,
        const CameraInfo::ConstSharedPtr & camera_info_msg)
    {
        // Lock the variables till the function terminates for concurrency purposes
        std::lock_guard<std::mutex> lock(data_mutex_);

        RCLCPP_DEBUG(this->get_logger(), "Saving data...");

        current_rgb_msg_ = *rgb_msg;
        current_depth_msg_ = *depth_msg;
        current_camera_info_msg_ = *camera_info_msg;

        // Wait for the transform to become available
        std::string target_frame = this->frame_id_;

        try {
            if (this->tf_buffer_->canTransform(
                target_frame, camera_info_msg->header.frame_id, camera_info_msg->header.stamp)) 
            {
                current_tf_ = this->tf_buffer_->lookupTransform(
                    target_frame, camera_info_msg->header.frame_id, camera_info_msg->header.stamp);
            }
            
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "[NBV][saveData] Could not transform %s to %s: %s", target_frame.c_str(), camera_info_msg->header.frame_id.c_str(), ex.what());
            return;
        } catch (const std::exception& e) {
            // This will catch standard exceptions
            RCLCPP_ERROR(this->get_logger(), "[NBV][saveData] %s",e.what());
        } catch (...) {
            // This will catch all other exceptions
            RCLCPP_ERROR(this->get_logger(), "[NBV][saveData] Generic error.");
        } 
        // Used to tell the main function that is time to execute
        data_received_ = true;
        // Used to block the wait, and to check the value of the above flag
        data_cond_.notify_one();

        RCLCPP_DEBUG(this->get_logger(), "Internal data updated.");

    }




    void NBVPipeline::NBVPipelineThread(){
        
        RCLCPP_INFO(this->get_logger(), "NBV pipeline started.");
        // set the rate for the main thread
	    rclcpp::Rate rate(15);


        // Wait for user input to start the pipeline with a service



        // Initialize data subscriber
        RCLCPP_INFO(this->get_logger(), "Creating data subscriber...");
        this->createDataSub();
        RCLCPP_INFO(this->get_logger(), "Data subscriber created");


        while (rclcpp::ok()){

            // Obtain data from the robot

            // Lock the variables till the function terminates, locking also the subsciber to save the current data
            std::unique_lock<std::mutex> lock(data_mutex_);
            // Wait intill notification: then check the value of data received
            data_cond_.wait(lock, [this]{ return this->data_received_; });
            data_received_ = false; // Reset the flag

            // Ontain data to send it to the segmentation server
            RCLCPP_INFO(this->get_logger(), "Getting data from subscriber...");

            Image working_rgb_msg;
            Image working_depth_msg;
            CameraInfo working_camera_info_msg;
            geometry_msgs::msg::TransformStamped working_tf;

            std::tie(
                working_rgb_msg, working_depth_msg, working_camera_info_msg, working_tf) = std::make_tuple(
                current_rgb_msg_, current_depth_msg_, current_camera_info_msg_, current_tf_);

            RCLCPP_INFO(this->get_logger(), "Data obtained.");




            // Create segmentation request for the server

            auto request = std::make_shared<fruit_picking_interfaces::srv::YOLOWorldSegmentation::Request>();
            request->image = working_rgb_msg;
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

            auto result = client_->async_send_request(request);




            // Wait for the response

            if (rclcpp::spin_until_future_complete(client_node_, result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "Response obtained.");
            
            }

        }


    }

}