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

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "cv_bridge/cv_bridge.h"

#include <depth_image_proc/conversions.hpp>
#include <depth_image_proc/point_cloud_xyzrgb.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <av_pointcloud/full_pointcloud.hpp>

namespace full_pointcloud {


/**
 * @brief Constructor for the FullPointcloud class.
 * 
 * This constructor initializes the FullPointcloud class, setting up various parameters and configurations required for the full point cloud processing.
 * It initializes the ROS 2 node, declares parameters, and sets up initial values for member variables.
 * 
 * 
 * @param options Node options for ROS 2 node initialization.
 */
FullPointcloud::FullPointcloud(const rclcpp::NodeOptions & options): 
    rclcpp::Node("FullPointcloud", options),
    centralizedArchitecture(false)
{
    
    centralizedArchitecture = this->declare_parameter("centralized_architecture", centralizedArchitecture);
    
    RCLCPP_INFO(this->get_logger(), "Full pointcloud parameters and variables initialized.");

}



/**
 * @brief Creates publishers and subscribers for the FullPointcloud class.
 * 
 * This function sets up the necessary publishers and subscribers for the FullPointcloud class. It reads parameters, initializes
 * synchronization policies, and sets up the subscribers for depth, RGB, and camera info topics. It also creates the publisher for
 * the point cloud data.
 * 
 */
void FullPointcloud::createPubSub()
{

    // Read parameters
    queue_size = this->declare_parameter<int>("queue_size", 5);
    use_exact_sync = this->declare_parameter<bool>("exact_sync", false);

    RCLCPP_INFO(this->get_logger(), "Creating full pointcloud subscribers...");

    // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
    if (use_exact_sync) {
        exact_sync_ = std::make_shared<ExactSynchronizer>(
        ExactSyncPolicy(queue_size),
        sub_depth_,
        sub_rgb_,
        sub_info_);
        exact_sync_->registerCallback(
        std::bind(
            &FullPointcloud::imageCb,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3));
    } else {
        sync_ = std::make_shared<Synchronizer>(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_);
        sync_->registerCallback(
        std::bind(
            &FullPointcloud::imageCb,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3));
    }

    // Subscribers

    if (0) {
        // TODO(ros2) Implement getNumSubscribers when rcl/rmw support it
        sub_depth_.unsubscribe();
        sub_rgb_.unsubscribe();
        sub_info_.unsubscribe();
    } else if (!sub_depth_.getSubscriber()) {
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
        this, "depth_registered/image_rect",
        depth_hints.getTransport(), rmw_qos_profile_default, sub_opts);

        // rgb uses normal ros transport hints.
        image_transport::TransportHints hints(this, "raw");
        sub_rgb_.subscribe(
        this, "rgb/image_rect_color",
        hints.getTransport(), rmw_qos_profile_default, sub_opts);
        sub_info_.subscribe(this, "rgb/camera_info");
    }

    RCLCPP_INFO(this->get_logger(), "Full pointcloud subscribers created.");

    RCLCPP_INFO(this->get_logger(), "Creating full pointcloud publishers...");


    // Publishers

    pub_point_cloud_ = create_publisher<PointCloud2>("points", rclcpp::SensorDataQoS());


    RCLCPP_INFO(this->get_logger(), "Full pointcloud publishers created.");

}



/**
 * @brief Callback function for processing synchronized depth, RGB, and camera info messages.
 * 
 * This function processes synchronized depth, RGB, and camera info messages to generate a point cloud. It checks for input consistency,
 * updates the camera model, resizes the RGB image if necessary, and converts the depth and RGB images to a point cloud.
 * 
 * Key steps include:
 * 1. **Check Input Consistency**: Ensure the depth and RGB images have matching frame IDs.
 * 2. **Update Camera Model**: Update the camera model using the camera info message.
 * 3. **Resize RGB Image (if needed)**: Resize the RGB image to match the depth image dimensions if necessary.
 * 4. **Set Color Encoding Offsets**: Determine the offsets for the color channels based on the RGB image encoding.
 * 5. **Initialize Point Cloud Message**: Create and initialize the point cloud message.
 * 6. **Convert Depth Image to Point Cloud**: Convert the depth image to a point cloud.
 * 7. **Convert RGB Image to Point Cloud**: Convert the RGB image to a point cloud.
 * 8. **Publish Point Cloud**: Publish the point cloud if the centralized architecture is not used.
 * 
 * @param depth_msg The depth image message.
 * @param rgb_msg_in The RGB image message.
 * @param info_msg The camera info message.
 * @return A shared pointer to the generated point cloud message.
 */
std::shared_ptr<sensor_msgs::msg::PointCloud2> FullPointcloud::imageCb(
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
                return nullptr;
        }
        cv_bridge::CvImage cv_rsz;
        cv_rsz.header = cv_ptr->header;
        cv_rsz.encoding = cv_ptr->encoding;
        cv::resize(
        cv_ptr->image.rowRange(0, depth_msg->height / ratio), cv_rsz.image,
        cv::Size(depth_msg->width, depth_msg->height));
        if ((rgb_msg->encoding == sensor_msgs::image_encodings::RGB8) ||
            (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8) ||
            (rgb_msg->encoding == sensor_msgs::image_encodings::MONO8)){
            rgb_msg = cv_rsz.toImageMsg();
        } 
        else {
            rgb_msg = cv_bridge::toCvCopy(cv_rsz.toImageMsg(), sensor_msgs::image_encodings::RGB8)->toImageMsg();
        }

        RCLCPP_ERROR(get_logger(), "Depth resolution (%ux%u) does not match RGB resolution (%ux%u)",
            depth_msg->width, depth_msg->height, rgb_msg->width, rgb_msg->height);
        return nullptr;
    } 
    else {
        rgb_msg = rgb_msg_in;
    }

    // Supported color encodings: RGB8, BGR8, MONO8
    int red_offset, green_offset, blue_offset, color_step;
    if (rgb_msg->encoding == sensor_msgs::image_encodings::RGB8) {
        red_offset = 0;
        green_offset = 1;
        blue_offset = 2;
        color_step = 3;
    } else if (rgb_msg->encoding == sensor_msgs::image_encodings::RGBA8) {
        red_offset = 0;
        green_offset = 1;
        blue_offset = 2;
        color_step = 4;
    } else if (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8) {
        red_offset = 2;
        green_offset = 1;
        blue_offset = 0;
        color_step = 3;
    } else if (rgb_msg->encoding == sensor_msgs::image_encodings::BGRA8) {
        red_offset = 2;
        green_offset = 1;
        blue_offset = 0;
        color_step = 4;
    } else if (rgb_msg->encoding == sensor_msgs::image_encodings::MONO8) {
        red_offset = 0;
        green_offset = 0;
        blue_offset = 0;
        color_step = 1;
    } else {
        try {
            rgb_msg = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::RGB8)->toImageMsg();
        } 
        catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(
                get_logger(), "Unsupported encoding [%s]: %s", rgb_msg->encoding.c_str(), e.what());
            return nullptr;
        }
        red_offset = 0;
        green_offset = 1;
        blue_offset = 2;
        color_step = 3;
    }

    auto cloud_msg = std::make_shared<PointCloud2>();
    cloud_msg->header = depth_msg->header;  // Use depth image time stamp
    cloud_msg->height = depth_msg->height;
    cloud_msg->width = depth_msg->width;
    cloud_msg->is_dense = false;
    cloud_msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    // Convert Depth Image to Pointcloud
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        depth_image_proc::convertDepth<uint16_t>(depth_msg, cloud_msg, model_);
    } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        depth_image_proc::convertDepth<float>(depth_msg, cloud_msg, model_);
    } else {
        RCLCPP_ERROR(
        get_logger(), "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
        return nullptr;
    }

    // Convert RGB
    if (rgb_msg->encoding == sensor_msgs::image_encodings::RGB8) {
        depth_image_proc::convertRgb(rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
    } else if (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8) {
        depth_image_proc::convertRgb(rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
    } else if (rgb_msg->encoding == sensor_msgs::image_encodings::BGRA8) {
        depth_image_proc::convertRgb(rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
    } else if (rgb_msg->encoding == sensor_msgs::image_encodings::RGBA8) {
        depth_image_proc::convertRgb(rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
    } else if (rgb_msg->encoding == sensor_msgs::image_encodings::MONO8) {
        depth_image_proc::convertRgb(rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
    } else {
        RCLCPP_ERROR(
        get_logger(), "RGB image has unsupported encoding [%s]", rgb_msg->encoding.c_str());
        return nullptr;
    }

    if (!centralizedArchitecture){
        pub_point_cloud_->publish(*cloud_msg);
    }

    return cloud_msg;
}


} 
