#pragma once
#ifndef _NBV_PIPELINE_HPP_
#define _NBV_PIPELINE_HPP_

#include <fruit_picking_pointcloud/full_pointcloud.hpp>
#include <fruit_picking_pointcloud/segmented_pointcloud.hpp>
#include <fruit_picking_octomap/extended_octomap_server.hpp>
#include <fruit_picking_interfaces/srv/yolo_world_segmentation.hpp>

// Definitions
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using ImageArray = fruit_picking_interfaces::msg::ImageArray;
using Confidence = fruit_picking_interfaces::msg::Confidence;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointcloudArray = fruit_picking_interfaces::msg::PointcloudArray;

// Synchronizer to get depth, rgb and camera info
using DataSyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image, CameraInfo>;
using DataSynchronizer = message_filters::Synchronizer<DataSyncPolicy>;



namespace nbv_pipeline{

    class NBVPipeline: public rclcpp::Node {

    protected:

        // Variables for saving nodes
        std::shared_ptr<full_pointcloud::FullPointcloud> pointcloud_node_;
        std::shared_ptr<segmented_pointcloud::SegmentedPointcloud> segmented_pointcloud_node_;
        std::shared_ptr<extended_octomap_server::ExtendedOctomapServer> extended_octomap_node_;
        std::shared_ptr<rclcpp::Node> client_node_;
        rclcpp::Client<fruit_picking_interfaces::srv::YOLOWorldSegmentation>::SharedPtr client_;

        // Data subscribers elements
        message_filters::Subscriber<Image> sub_depth_, sub_rgb_;
        message_filters::Subscriber<CameraInfo> sub_camera_info_;
        std::shared_ptr<DataSynchronizer> data_sync_;
        int queue_size_;

        // Mutex
        std::mutex data_mutex_;
        bool data_received_;
        std::condition_variable data_cond_;


        // Variable containing current and realtime depth, rgb, camera info and tf
        Image::ConstSharedPtr current_rgb_msg_;
        Image::ConstSharedPtr current_depth_msg_;
        CameraInfo::ConstSharedPtr current_camera_info_msg_;
        std::string frame_id_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::msg::TransformStamped::ConstSharedPtr current_tf_;

        // Variables for segmentation
        std::string prompt_;
        float confidence_threshold_;
        float nms_confidence_threshold_;


        // Variables containing masks images array, confidences, classes
        ImageArray::ConstSharedPtr masks_images_array_;
        Image::ConstSharedPtr merged_masks_image_;
        Confidence::ConstSharedPtr confidences_;


        // Variable specifing to use the full pointcloud or a partial
        bool usePartialPointcloud_;
        std::shared_ptr<PointCloud2> fullPointcloud_;
        std::shared_ptr<PointCloud2> partialPointcloud_;
        std::shared_ptr<PointcloudArray> segmentedPointcloudArray_;


        // Function that create the data subscriber
        void createDataSub();

        // Callback called by the synchronizer that saves and updates the depth, rgb and camera info
        void saveData(
            const Image::ConstSharedPtr & rgb_msg,
            const Image::ConstSharedPtr & depth_msg,
            const CameraInfo::ConstSharedPtr & camera_info_msg);


    public:

        NBVPipeline(
            std::shared_ptr<full_pointcloud::FullPointcloud> pointcloud_creator,
            std::shared_ptr<segmented_pointcloud::SegmentedPointcloud> segmented_pointcloud_creator,
            std::shared_ptr<extended_octomap_server::ExtendedOctomapServer> extended_octomap_creator,
            std::shared_ptr<rclcpp::Node> segmentationClientNode,
            const rclcpp::NodeOptions &,
            const std::string = "nbv_pipeline");

        void NBVPipelineThread();
        
    };
    

} // end namespace main_pipeline
#endif