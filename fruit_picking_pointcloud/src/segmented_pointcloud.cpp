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

#include <fruit_picking_pointcloud/segmented_pointcloud.hpp>

namespace segmented_pointcloud{

SegmentedPointcloud::SegmentedPointcloud(const rclcpp::NodeOptions &options) : rclcpp::Node("SegmentedPointcloud", options){

    // Read parameters
    queue_size = this->declare_parameter<int>("queue_size", 5);
    use_exact_sync = this->declare_parameter<bool>("exact_sync", false);

    // used to specify if a single segmented pointcloud of an array of segmented pointclouds have to be published
    publishPointcloudsArray = this->declare_parameter("publish_pointclouds_array", publishPointcloudsArray);
    publishSinglePointcloud = this->declare_parameter("publish_single_pointcloud", publishSinglePointcloud);

    
    RCLCPP_INFO(this->get_logger(), "Segmented pointcloud parameters and variables initialized.");

}

void SegmentedPointcloud::createPubSub()
{

    RCLCPP_INFO(this->get_logger(), "Creating segmented pointcloud subscribers...");

    // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
    // Each case has a case for the pointclouds array and one for a single pointcloud
    if (use_exact_sync) {
        if (publishPointcloudsArray){
            exact_sync_array_ = std::make_shared<ExactSynchronizerArray>(ExactSyncPolicyArray(queue_size), sub_depth_, sub_rgb_array_, sub_info_, sub_conf_);
            exact_sync_array_->registerCallback(
                std::bind(
                    &SegmentedPointcloud::imageArrayCb,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3,
                    std::placeholders::_4));
            
        }
        if (publishSinglePointcloud){
            exact_sync_ = std::make_shared<ExactSynchronizer>(ExactSyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_);
            exact_sync_->registerCallback(
                std::bind(
                    &SegmentedPointcloud::imageCb,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3));
        }
          
    } 
    else {
        if (publishPointcloudsArray){
            sync_array_ = std::make_shared<SynchronizerArray>(SyncPolicyArray(queue_size), sub_depth_, sub_rgb_array_, sub_info_, sub_conf_);
            sync_array_->registerCallback(
                std::bind(
                    &SegmentedPointcloud::imageArrayCb,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3,
                    std::placeholders::_4));            
        }
        if (publishSinglePointcloud){
            sync_ = std::make_shared<Synchronizer>(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_);
            sync_->registerCallback(
                std::bind(
                    &SegmentedPointcloud::imageCb,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3));
        }
    }

    // Subscribers

    if (!sub_depth_.getSubscriber()) {

        sub_depth_.subscribe(this, "depth_image");
        sub_info_.subscribe(this, "depth_image_camera_info");

        if (publishPointcloudsArray){
            sub_rgb_array_.subscribe(this, "rgb_images_array");            
            sub_conf_.subscribe(this, "confidences");
        }
        if (publishSinglePointcloud){
            sub_rgb_.subscribe(this, "rgb_image");
        }
    }

    RCLCPP_INFO(this->get_logger(), "Segmented pointcloud subscribers created.");

    RCLCPP_INFO(this->get_logger(), "Creating segmented pointcloud publishers...");


    // Publishers

    if (publishPointcloudsArray){
        pub_point_cloud_array_ = create_publisher<PointCloud2Array>("segmented_pointclouds_array", rclcpp::SensorDataQoS());
    }
    if (publishSinglePointcloud){
        pub_point_cloud_ = create_publisher<PointCloud2>("segmented_pointcloud", rclcpp::SensorDataQoS());
    }

    RCLCPP_INFO(this->get_logger(), "Segmented pointcloud publishers created.");

}

void SegmentedPointcloud::imageCb(
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

    // Create an empty pointcloud message that will be published at the end
    auto pointcloud_msg = std::make_shared<PointCloud2>();

    // Convert Depth Image message to Pointcloud message such that the final pointcloud contain only the segmented points
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        convertDepth<uint16_t>(*depth_msg, *rgb_msg, *pointcloud_msg, model_);
    } 
    else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        convertDepth<float>(*depth_msg, *rgb_msg, *pointcloud_msg, model_);
    } 
    else {
        RCLCPP_ERROR(
            get_logger(), "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
        return;
    }

    // Fill header of pointcloud message
    pointcloud_msg->header = depth_msg->header;
    pointcloud_msg->is_dense = false;
    pointcloud_msg->is_bigendian = false;

    pub_point_cloud_->publish(*pointcloud_msg);
    RCLCPP_INFO(this->get_logger(), "Pointcloud published.");

}

void SegmentedPointcloud::imageArrayCb(
    const Image::ConstSharedPtr & depth_msg,
    const ImageArray::ConstSharedPtr & rgb_array_in,
    const CameraInfo::ConstSharedPtr & info_msg,
    const std::shared_ptr<const Confidence> & conf_msg)
{
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

    // Create an empty pointclouds array message that will be published at the end
    auto pointcloud_array = std::make_shared<PointCloud2Array>();

    // Fill the pointclouds array message
    pointcloud_array->header = depth_msg->header;
    pointcloud_array->confidences = conf_msg->data;
    pointcloud_array->pointclouds = std::vector<PointCloud2>(rgb_array_in->images.size());
    pointcloud_array->semantic_class = conf_msg->semantic_class;

    for (size_t i = 0; i < rgb_array_in->images.size(); ++i){

        // Define the rgb message containing a segmentation
        auto& rgb_msg_in = rgb_array_in->images[i];
        // Define the segmented pointcloud that will contain a segmentation
        auto pointcloud_msg = std::make_shared<PointCloud2>();

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

        // Convert Depth Image message to Pointcloud message such that the final pointcloud contain only the segmented points
        if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            convertDepth<uint16_t>(*depth_msg, rgb_msg, *pointcloud_msg, model_);
        } 
        else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            convertDepth<float>(*depth_msg, rgb_msg, *pointcloud_msg, model_);
        } 
        else {
            RCLCPP_ERROR(get_logger(), "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
            return;
        }

        // Fill header of pointcloud message
        pointcloud_msg->header = depth_msg->header;
        pointcloud_msg->is_dense = true; // true because there are not invalid (NaN) points
        pointcloud_msg->is_bigendian = false;

        pointcloud_array->pointclouds[i] = *pointcloud_msg;

        // Calculate the number of points in the point cloud
        RCLCPP_INFO(this->get_logger(), "The final size of the pointcloud is %zu", pointcloud_msg->data.size());

    }

    pub_point_cloud_array_->publish(*pointcloud_array);
    RCLCPP_INFO(this->get_logger(), "Pointcloud array published.");  


    // Publish the first pointcloud in the pointclouds array (used mostly to debug)
    // if (publishSinglePointcloud){
    //     // Create an empty pointclouds array message that will be published at the end
    //     auto single_pointcloud = pointcloud_array->pointclouds[0];
    //     pub_point_cloud_->publish(single_pointcloud);
    //     RCLCPP_INFO(this->get_logger(), "Pointcloud published.");  
    // }  
}

    
} // namespace