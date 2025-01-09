#include <av_pointcloud/full_pointcloud.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<full_pointcloud::FullPointcloud>(rclcpp::NodeOptions());
    node->createPubSub();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}