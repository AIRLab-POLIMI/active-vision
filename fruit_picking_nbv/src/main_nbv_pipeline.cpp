#include <fruit_picking_nbv/nbv_pipeline.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto PointcloudCreator = std::make_shared<full_pointcloud::FullPointcloud>(rclcpp::NodeOptions());
    auto SegmentedPointcloudCreator = std::make_shared<segmented_pointcloud::SegmentedPointcloud>(rclcpp::NodeOptions());
    auto ExtendedOctomapCreator = std::make_shared<extended_octomap_server::ExtendedOctomapServer>(rclcpp::NodeOptions());

	// Segmentation client node
	auto SegmentationClientNode = rclcpp::Node::make_shared("yolo_world_client");

    // Create NBV pipeline object
    auto Pipeline = std::make_shared<nbv_pipeline::NBVPipeline>(
        PointcloudCreator, 
        SegmentedPointcloudCreator, 
        ExtendedOctomapCreator, 
		SegmentationClientNode,
        rclcpp::NodeOptions()
    );


    // asynchronous multi-threaded executor for spinning the nodes in separate threads
	rclcpp::executors::MultiThreadedExecutor executor;
	auto main_thread = std::make_unique<std::thread>([&executor,
													  &PointcloudCreator,
													  &SegmentedPointcloudCreator,
													  &ExtendedOctomapCreator,
													  &Pipeline]() {
		executor.add_node(PointcloudCreator->get_node_base_interface());
		executor.add_node(SegmentedPointcloudCreator->get_node_base_interface());
		executor.add_node(ExtendedOctomapCreator->get_node_base_interface());
		executor.add_node(Pipeline->get_node_base_interface());
		executor.spin();
	});

    // start the main thread for the grasp pose estimator node to estimate the grasp pose from object coordinates
	std::thread pipeline_thread = std::thread(
		&nbv_pipeline::NBVPipeline::NBVPipelineThread, Pipeline);
	pipeline_thread.detach();

	main_thread->join();
	pipeline_thread.join();

	rclcpp::shutdown();
    return 0;
}