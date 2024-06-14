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


#pragma once
#ifndef _SEGMENTED_POINTCLOUD_HPP_
#define _SEGMENTED_POINTCLOUD_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "depth_image_proc/visibility.h"
#include <depth_image_proc/conversions.hpp>
#include <depth_image_proc/point_cloud_xyzrgb.hpp>
#include "image_geometry/pinhole_camera_model.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cv_bridge/cv_bridge.h"

#include "rclcpp_components/register_node_macro.hpp"

#include "fruit_picking_interfaces/msg/pointcloud_array.hpp"
#include "fruit_picking_interfaces/msg/image_array.hpp"
#include "fruit_picking_interfaces/msg/confidence.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>


namespace segmented_pointcloud{
    
    class SegmentedPointcloud : public rclcpp::Node{

    protected:

        using PointCloud2 = sensor_msgs::msg::PointCloud2;
        using PointCloud2Array = fruit_picking_interfaces::msg::PointcloudArray;
        using Image = sensor_msgs::msg::Image;
        using ImageArray = fruit_picking_interfaces::msg::ImageArray;
        using CameraInfo = sensor_msgs::msg::CameraInfo;
        using Confidence = fruit_picking_interfaces::msg::Confidence;

        // Subscriptions
        message_filters::Subscriber<Image> sub_depth_, sub_rgb_;
        message_filters::Subscriber<ImageArray> sub_rgb_array_;
        message_filters::Subscriber<CameraInfo> sub_info_;
        message_filters::Subscriber<Confidence> sub_conf_;
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image, CameraInfo>;
        using ExactSyncPolicy = message_filters::sync_policies::ExactTime<Image, Image, CameraInfo>;
        using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
        using ExactSynchronizer = message_filters::Synchronizer<ExactSyncPolicy>;
        using SyncPolicyArray = message_filters::sync_policies::ApproximateTime<Image, ImageArray, CameraInfo, Confidence>;
        using ExactSyncPolicyArray = message_filters::sync_policies::ExactTime<Image, ImageArray, CameraInfo, Confidence>;
        using SynchronizerArray = message_filters::Synchronizer<SyncPolicyArray>;
        using ExactSynchronizerArray = message_filters::Synchronizer<ExactSyncPolicyArray>;
        std::shared_ptr<Synchronizer> sync_;
        std::shared_ptr<ExactSynchronizer> exact_sync_;
        std::shared_ptr<SynchronizerArray> sync_array_;
        std::shared_ptr<ExactSynchronizerArray> exact_sync_array_;

        // Publications
        int queue_size;
        bool use_exact_sync;
        rclcpp::Publisher<PointCloud2Array>::SharedPtr pub_point_cloud_array_;
        rclcpp::Publisher<PointCloud2>::SharedPtr pub_point_cloud_;

        image_geometry::PinholeCameraModel model_;

        bool publishPointcloudsArray;
        bool publishSinglePointcloud;

        bool centralizedArchitecture;

        
        template<typename T>
        void convertDepth(
            const sensor_msgs::msg::Image & depth_msg,
            const sensor_msgs::msg::Image & rgb_msg,
            sensor_msgs::msg::PointCloud2 & cloud_msg,
            const image_geometry::PinholeCameraModel & model)
        {
            // Use correct principal point from calibration
            float center_x = model.cx();
            float center_y = model.cy();

            // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
            double unit_scaling = depth_image_proc::DepthTraits<T>::toMeters(T(1));
            float constant_x = unit_scaling / model.fx();
            float constant_y = unit_scaling / model.fy();

            // Create an empty pcl::PointCloud<pcl::PointXYZ>
            pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

            const T* depth_row = reinterpret_cast<const T*>(&depth_msg.data[0]);
            const uint8_t* rgb_row = &rgb_msg.data[0];
            int row_step = depth_msg.step / sizeof(T);
            for (int v = 0; v < static_cast<int>(depth_msg.height); ++v, depth_row += row_step, rgb_row += rgb_msg.step) {
                for (int u = 0; u < static_cast<int>(depth_msg.width); ++u) {
                    T depth = depth_row[u];

                    // Get the RGB values
                    uint8_t r = rgb_row[u * 3];
                    uint8_t g = rgb_row[u * 3 + 1];
                    uint8_t b = rgb_row[u * 3 + 2];

                    // Check if the RGB value is white (assuming 255,255,255 is white)
                    if (r == 255 && g == 255 && b == 255) {
                        continue; // Skip this point
                    }

                    // Missing points denoted by NaNs
                    if (!depth_image_proc::DepthTraits<T>::valid(depth)) {
                        continue;
                    }

                    // Fill in XYZ
                    pcl::PointXYZ point;
                    point.x = (u - center_x) * depth * constant_x;
                    point.y = (v - center_y) * depth * constant_y;
                    point.z = depth_image_proc::DepthTraits<T>::toMeters(depth);
                    pcl_cloud.points.push_back(point);
                }
            }

            // Convert the pcl::PointCloud to a sensor_msgs::PointCloud2
            pcl::toROSMsg(pcl_cloud, cloud_msg);
            cloud_msg.height = 1;
            cloud_msg.width = pcl_cloud.points.size();

            RCLCPP_INFO(this->get_logger(), "[CONVERT_DEPTH] The number of points of the PCL pointcloud is %zu", pcl_cloud.points.size());

        }
    
    public:
        
        SegmentedPointcloud(const rclcpp::NodeOptions& options);

        void createPubSub();

        std::shared_ptr<fruit_picking_interfaces::msg::PointcloudArray> imageArrayCb(
            const Image::ConstSharedPtr & depth_msg,
            const ImageArray::ConstSharedPtr & rgb_msg,
            const CameraInfo::ConstSharedPtr & info_msg,
            const std::shared_ptr<const Confidence> & conf_msg);

        std::shared_ptr<sensor_msgs::msg::PointCloud2> imageCb(
            const Image::ConstSharedPtr & depth_msg,
            const Image::ConstSharedPtr & rgb_msg_in,
            const CameraInfo::ConstSharedPtr & info_msg);

    };
    

} // end namespace extended_octomap
#endif