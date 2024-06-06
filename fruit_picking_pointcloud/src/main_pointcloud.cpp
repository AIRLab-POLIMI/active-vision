#include <depth_image_proc/point_cloud_xyzrgb.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<depth_image_proc::PointCloudXyzrgbNode>(rclcpp::NodeOptions());

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}