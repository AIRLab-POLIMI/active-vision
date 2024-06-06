#pragma once
#ifndef _MAIN_PIPELINE_HPP_
#define _MAIN_PIPELINE_HPP_

#include <fruit_picking_pointcloud/segmented_pointcloud.hpp>
#include <fruit_picking_octomap/extended_octomap_server.hpp>
#include <depth_image_proc/point_cloud_xyzrgb.hpp>

namespace main_pipeline{

    class MainPipeline: public rclcpp::Node {

    protected:

        
        

    public:

        explicit MainPipeline(
            const rclcpp::NodeOptions &,
            const std::string = "main_pipeline");
        
    };
    

} // end namespace main_pipeline
#endif