#pragma once
#ifndef _NBV_PIPELINE_HPP_
#define _NBV_PIPELINE_HPP_

#include <fruit_picking_pointcloud/segmented_pointcloud.hpp>
#include <fruit_picking_octomap/extended_octomap_server.hpp>
#include <depth_image_proc/point_cloud_xyzrgb.hpp>

namespace nbv_pipeline{

    class NBVPipeline: public rclcpp::Node {

    protected:

        
        

    public:

        NBVPipeline(
            std::shared_ptr<depth_image_proc::PointCloudXyzrgbNode> pointcloud_creator,
            std::shared_ptr<segmented_pointcloud::SegmentedPointcloud> segmented_pointcloud_creator,
            std::shared_ptr<extended_octomap_server::ExtendedOctomapServer> extended_octomap_creator,
            const rclcpp::NodeOptions &,
            const std::string = "nbv_pipeline");

        void NBVPipelineThread();
        
    };
    

} // end namespace main_pipeline
#endif