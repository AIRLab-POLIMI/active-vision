#include <fruit_picking_nbv/nbv_pipeline.hpp>

namespace nbv_pipeline{
    
    NBVPipeline::NBVPipeline(
        std::shared_ptr<depth_image_proc::PointCloudXyzrgbNode> pointcloud_creator,
        std::shared_ptr<segmented_pointcloud::SegmentedPointcloud> segmented_pointcloud_creator,
        std::shared_ptr<extended_octomap_server::ExtendedOctomapServer> extended_octomap_creator,
        const rclcpp::NodeOptions &options, 
        const std::string node_name): 
        Node(node_name, options)
    {
        RCLCPP_INFO(this->get_logger(), "NBV pipeline constructor started.");
    }


    void NBVPipeline::NBVPipelineThread(){
        
        RCLCPP_INFO(this->get_logger(), "NBV pipeline started.");

    }

}