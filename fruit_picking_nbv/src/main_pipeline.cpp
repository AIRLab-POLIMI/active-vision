#include <fruit_picking_nbv/main_pipeline.hpp>

namespace main_pipeline{
    
    MainPipeline::MainPipeline(const rclcpp::NodeOptions &options, const std::string node_name): Node(node_name, options){
        RCLCPP_INFO(this->get_logger(), "Main pipeline constructor started.");
    }

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<main_pipeline::MainPipeline>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}