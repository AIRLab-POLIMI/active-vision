#include <fruit_picking_pointcloud/segmented_pointcloud.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<segmented_pointcloud::SegmentedPointcloud>(rclcpp::NodeOptions());
    node->createPubSub();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}