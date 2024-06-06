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

        explicit NBVPipeline(
            const rclcpp::NodeOptions &,
            const std::string = "nbv_pipeline");
        
    };
    

} // end namespace main_pipeline
#endif