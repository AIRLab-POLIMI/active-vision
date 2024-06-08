#include <fruit_picking_octomap/extended_octomap_server.hpp>


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<extended_octomap_server::ExtendedOctomapServer>(rclcpp::NodeOptions(), "extended_octomap_server");
    node->createPubSub();
    node->createVisualizations();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}