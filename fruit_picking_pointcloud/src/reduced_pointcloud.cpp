// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <fruit_picking_pointcloud/reduced_pointcloud.hpp>

namespace reduced_pointcloud{

ReducedPointcloud::ReducedPointcloud(const rclcpp::NodeOptions &options) : rclcpp::Node("ReducedPointcloud", options){

    // Read parameters
    int queue_size = this->declare_parameter<int>("queue_size", 5);
    bool use_exact_sync = this->declare_parameter<bool>("exact_sync", false);
    publishPointcloudArray = this->declare_parameter("publish_pointcloud_array", publishPointcloudArray);
    

    // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
    if (use_exact_sync) {
        if (publishPointcloudArray){
            RCLCPP_INFO(this->get_logger(), "[REDUCED POINTCLOUD] Using exact sync for array...");
            exact_sync_array_ = std::make_shared<ExactSynchronizerArray>(ExactSyncPolicyArray(queue_size), sub_depth_, sub_rgb_array_, sub_info_, sub_conf_);
            exact_sync_array_->registerCallback(
                std::bind(
                    &ReducedPointcloud::imageArrayCb,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3,
                    std::placeholders::_4));
            
        }
        else{
            RCLCPP_INFO(this->get_logger(), "[REDUCED POINTCLOUD] Using exact sync...");
            exact_sync_ = std::make_shared<ExactSynchronizer>(ExactSyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_);
            exact_sync_->registerCallback(
                std::bind(
                    &ReducedPointcloud::imageCb,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3));
        }
          
    } 
    else {
        if (publishPointcloudArray){
            RCLCPP_INFO(this->get_logger(), "[REDUCED POINTCLOUD] Using apporximate sync for array...");
            sync_array_ = std::make_shared<SynchronizerArray>(SyncPolicyArray(queue_size), sub_depth_, sub_rgb_array_, sub_info_, sub_conf_);
            sync_array_->registerCallback(
                std::bind(
                    &ReducedPointcloud::imageArrayCb,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3,
                    std::placeholders::_4));            
        }
        else{
            RCLCPP_INFO(this->get_logger(), "[REDUCED POINTCLOUD] Using approximate sync...");
            sync_ = std::make_shared<Synchronizer>(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_);
            sync_->registerCallback(
                std::bind(
                    &ReducedPointcloud::imageCb,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3));
        }
    }

    connectCb();
    std::lock_guard<std::mutex> lock(connect_mutex_);
    if (publishPointcloudArray){
        RCLCPP_INFO(this->get_logger(), "Creating publisher for pointcloud array...");
        pub_point_cloud_array_ = create_publisher<PointCloud2Array>("reduced/points_array", rclcpp::SensorDataQoS());
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Creating publisher for pointcloud...");
        pub_point_cloud_ = create_publisher<PointCloud2>("reduced/points", rclcpp::SensorDataQoS());
    }

}

void ReducedPointcloud::connectCb()
{
    std::lock_guard<std::mutex> lock(connect_mutex_);
    // TODO(ros2) Implement getNumSubscribers when rcl/rmw support it
    // if (pub_point_cloud_->getNumSubscribers() == 0)
    if (0) {
        // TODO(ros2) Implement getNumSubscribers when rcl/rmw support it
        sub_depth_.unsubscribe();
        sub_rgb_.unsubscribe();
        if (publishPointcloudArray){
            sub_rgb_array_.unsubscribe();
        }
        sub_info_.unsubscribe();
    } 
    else if (!sub_depth_.getSubscriber()) {
        // parameter for depth_image_transport hint
        std::string depth_image_transport_param = "depth_image_transport";
        image_transport::TransportHints depth_hints(this, "raw", depth_image_transport_param);

        rclcpp::SubscriptionOptions sub_opts;
        // Update the subscription options to allow reconfigurable qos settings.
        sub_opts.qos_overriding_options = rclcpp::QosOverridingOptions {
          {
            // Here all policies that are desired to be reconfigurable are listed.
            rclcpp::QosPolicyKind::Depth,
            rclcpp::QosPolicyKind::Durability,
            rclcpp::QosPolicyKind::History,
            rclcpp::QosPolicyKind::Reliability,
          }};

        // depth image can use different transport.(e.g. compressedDepth)
        sub_depth_.subscribe(
            this, "reduced/depth_registered/image_rect");
        // rgb array sub uses not ros transport hints.
        if (publishPointcloudArray){
            sub_rgb_array_.subscribe(this, "reduced/rgb/image_rect_color_array");            
            sub_conf_.subscribe(this, "reduced/confidences");
        }
        // rgb uses normal ros transport hints.
        image_transport::TransportHints hints(this, "raw");
        sub_rgb_.subscribe(
            this, "reduced/rgb/image_rect_color");
        sub_info_.subscribe(this, "reduced/rgb/camera_info");
    }
    RCLCPP_INFO(this->get_logger(), "[REDUCED POINTCLOUD] Connection done.");

}

void ReducedPointcloud::imageCb(
    const Image::ConstSharedPtr & depth_msg,
    const Image::ConstSharedPtr & rgb_msg_in,
    const CameraInfo::ConstSharedPtr & info_msg)
{
    // Check for bad inputs
    if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            10000,  // 10 seconds
            "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
            depth_msg->header.frame_id.c_str(), rgb_msg_in->header.frame_id.c_str());
    }

    // Update camera model
    model_.fromCameraInfo(info_msg);

    // Check if the input image has to be resized
    Image::ConstSharedPtr rgb_msg = rgb_msg_in;
    if (depth_msg->width != rgb_msg->width || depth_msg->height != rgb_msg->height) {
        CameraInfo info_msg_tmp = *info_msg;
        info_msg_tmp.width = depth_msg->width;
        info_msg_tmp.height = depth_msg->height;
        float ratio = static_cast<float>(depth_msg->width) / static_cast<float>(rgb_msg->width);
        info_msg_tmp.k[0] *= ratio;
        info_msg_tmp.k[2] *= ratio;
        info_msg_tmp.k[4] *= ratio;
        info_msg_tmp.k[5] *= ratio;
        info_msg_tmp.p[0] *= ratio;
        info_msg_tmp.p[2] *= ratio;
        info_msg_tmp.p[5] *= ratio;
        info_msg_tmp.p[6] *= ratio;
        model_.fromCameraInfo(info_msg_tmp);

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding);
        } 
        catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv_bridge::CvImage cv_rsz;
        cv_rsz.header = cv_ptr->header;
        cv_rsz.encoding = cv_ptr->encoding;
        cv::resize(
            cv_ptr->image.rowRange(0, depth_msg->height / ratio), cv_rsz.image,
            cv::Size(depth_msg->width, depth_msg->height));
        if ((rgb_msg->encoding == sensor_msgs::image_encodings::RGB8) ||
            (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8) ||
            (rgb_msg->encoding == sensor_msgs::image_encodings::MONO8))
        {
            rgb_msg = cv_rsz.toImageMsg();
        } 
        else {
            rgb_msg =
                cv_bridge::toCvCopy(cv_rsz.toImageMsg(), sensor_msgs::image_encodings::RGB8)->toImageMsg();
        }

        RCLCPP_ERROR(
            get_logger(), "Depth resolution (%ux%u) does not match RGB resolution (%ux%u)",
                depth_msg->width, depth_msg->height, rgb_msg->width, rgb_msg->height);
        return;
    } 
    else {
        rgb_msg = rgb_msg_in;
    }

   

    auto cloud_msg = std::make_shared<PointCloud2>();

    // sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg); // not used since it will be converted into a pcl cloud
    // pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    // Convert Depth Image to Pointcloud
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        convertDepth<uint16_t>(depth_msg, *rgb_msg, *cloud_msg, model_);
    } 
    else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        convertDepth<float>(depth_msg, *rgb_msg, *cloud_msg, model_);
    } 
    else {
        RCLCPP_ERROR(
            get_logger(), "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
        return;
    }

    cloud_msg->header = depth_msg->header;
    cloud_msg->is_dense = false;
    cloud_msg->is_bigendian = false;

    pub_point_cloud_->publish(*cloud_msg);
    RCLCPP_INFO(this->get_logger(), "[REDUCED POINTCLOUD] Pointcloud published.");

}

void ReducedPointcloud::imageArrayCb(
    const Image::ConstSharedPtr & depth_msg,
    const ImageArray::ConstSharedPtr & rgb_array_in,
    const CameraInfo::ConstSharedPtr & info_msg,
    const std::shared_ptr<const Confidence> & conf_msg)
{
    RCLCPP_INFO(this->get_logger(), "[REDUCED POINTCLOUD] Image Array CB activated.");

    // Check for bad inputs through all the rgb images
    for (const auto& rgb_msg_in : rgb_array_in->images){
        if (depth_msg->header.frame_id != rgb_msg_in.header.frame_id) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                10000,  // 10 seconds
                "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
                depth_msg->header.frame_id.c_str(), rgb_msg_in.header.frame_id.c_str());
        }
    }

    // Update camera model
    model_.fromCameraInfo(info_msg);

    auto cloud_array = std::make_shared<PointCloud2Array>();
    cloud_array->header = depth_msg->header;
    cloud_array->confidences = conf_msg->data;
    cloud_array->pointclouds = std::vector<PointCloud2>(rgb_array_in->images.size());

    auto cloud = std::make_shared<PointCloud2>();

    pcl::PointCloud<pcl::PointXYZ> full_pcl_cloud;


    for (size_t i = 0; i < rgb_array_in->images.size(); ++i){

        auto& rgb_msg_in = rgb_array_in->images[i];
        auto cloud_msg = std::make_shared<PointCloud2>();


        // Check if the input images have to be resized
        Image rgb_msg = rgb_msg_in;
        if (depth_msg->width != rgb_msg.width || depth_msg->height != rgb_msg.height) {
            CameraInfo info_msg_tmp = *info_msg;
            info_msg_tmp.width = depth_msg->width;
            info_msg_tmp.height = depth_msg->height;
            float ratio = static_cast<float>(depth_msg->width) / static_cast<float>(rgb_msg.width);
            info_msg_tmp.k[0] *= ratio;
            info_msg_tmp.k[2] *= ratio;
            info_msg_tmp.k[4] *= ratio;
            info_msg_tmp.k[5] *= ratio;
            info_msg_tmp.p[0] *= ratio;
            info_msg_tmp.p[2] *= ratio;
            info_msg_tmp.p[5] *= ratio;
            info_msg_tmp.p[6] *= ratio;
            model_.fromCameraInfo(info_msg_tmp);

            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(rgb_msg, rgb_msg.encoding);
            } 
            catch (cv_bridge::Exception & e) {
                RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            cv_bridge::CvImage cv_rsz;
            cv_rsz.header = cv_ptr->header;
            cv_rsz.encoding = cv_ptr->encoding;
            cv::resize(
            cv_ptr->image.rowRange(0, depth_msg->height / ratio), cv_rsz.image,
            cv::Size(depth_msg->width, depth_msg->height));
            if ((rgb_msg.encoding == sensor_msgs::image_encodings::RGB8) ||
                (rgb_msg.encoding == sensor_msgs::image_encodings::BGR8) ||
                (rgb_msg.encoding == sensor_msgs::image_encodings::MONO8))
            {
                rgb_msg = *(cv_rsz.toImageMsg());
            } 
            else {
                rgb_msg = *(cv_bridge::toCvCopy(cv_rsz.toImageMsg(), sensor_msgs::image_encodings::RGB8)->toImageMsg());
            }

            RCLCPP_ERROR(
                get_logger(), "Depth resolution (%ux%u) does not match RGB resolution (%ux%u)",
                depth_msg->width, depth_msg->height, rgb_msg.width, rgb_msg.height);
                return;
        } 
        else{
            rgb_msg = rgb_msg_in;
        }     

        // sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
        // pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

        // Convert Depth Image to Pointcloud
        if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            convertDepth<uint16_t>(depth_msg, rgb_msg, *cloud_msg, model_);
        } 
        else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            convertDepth<float>(depth_msg, rgb_msg, *cloud_msg, model_);
        } 
        else {
            RCLCPP_ERROR(get_logger(), "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*cloud_msg, pcl_cloud);

        full_pcl_cloud += pcl_cloud;

        cloud_msg->header = depth_msg->header;
        cloud_msg->is_dense = false;
        cloud_msg->is_bigendian = false;

        cloud_array->pointclouds[i] = *cloud_msg;

        // Calculate the number of points in the point cloud
        RCLCPP_INFO(this->get_logger(), "The final size of the pointcloud is %zu", cloud_msg->data.size());

    }

    pub_point_cloud_array_->publish(*cloud_array);
    RCLCPP_INFO(this->get_logger(), "[REDUCED POINTCLOUD][IMAGEARRAYCB] Pointcloud array published.");

    pcl::toROSMsg(full_pcl_cloud, *cloud);
    cloud->header = depth_msg->header;
    cloud->height = 1;
    cloud->width = full_pcl_cloud.points.size();
    cloud->is_dense = false;
    cloud->is_bigendian = false;

    
    RCLCPP_INFO(this->get_logger(), "The final size of the full pointcloud is %zu", cloud->data.size());

    // pub_point_cloud_->publish(*cloud);
    RCLCPP_INFO(this->get_logger(), "[REDUCED POINTCLOUD][IMAGEARRAYCB] Full pointcloud published.");

    
}
    
} // namespace


// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(reduced_pointcloud::ReducedPointcloud)
