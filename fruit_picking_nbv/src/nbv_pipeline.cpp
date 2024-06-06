#include <fruit_picking_nbv/nbv_pipeline.hpp>

namespace nbv_pipeline{
    
    NBVPipeline::NBVPipeline(const rclcpp::NodeOptions &options, const std::string node_name): Node(node_name, options){
        RCLCPP_INFO(this->get_logger(), "NBV pipeline constructor started.");
    }

}