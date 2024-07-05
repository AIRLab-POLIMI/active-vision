#include <fruit_picking_planning/active_vision_nbv_planning_pipeline.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

	auto node_options = rclcpp::NodeOptions();
	auto MoveIt2APICreator = std::make_shared<MoveIt2APIs>(node_options);
    auto PointcloudCreator = std::make_shared<full_pointcloud::FullPointcloud>(node_options);
    auto SegmentedPointcloudCreator = std::make_shared<segmented_pointcloud::SegmentedPointcloud>(node_options);
    auto ExtendedOctomapCreator = std::make_shared<extended_octomap_server::ExtendedOctomapServer>(node_options);

	// Segmentation client node
	auto SegmentationClientNode = rclcpp::Node::make_shared("yolo_world_client");

    // Create active vision pipeline object
    auto Pipeline = std::make_shared<active_vision_nbv_planning_pipeline::ActiveVisionNbvPlanningPipeline>(
		MoveIt2APICreator,
        PointcloudCreator, 
        SegmentedPointcloudCreator, 
        ExtendedOctomapCreator, 
		SegmentationClientNode,
        node_options
    );
	// Pipeline->get_logger().set_level(rclcpp::Logger::Level::Debug);


    // asynchronous multi-threaded executor for spinning the nodes in separate threads
	rclcpp::executors::MultiThreadedExecutor executor;
	auto main_thread = std::make_unique<std::thread>([&executor,
													  &MoveIt2APICreator,
													  &PointcloudCreator,
													  &SegmentedPointcloudCreator,
													  &ExtendedOctomapCreator,
													  &Pipeline]() {
		executor.add_node(MoveIt2APICreator->get_node_base_interface());
		executor.add_node(PointcloudCreator->get_node_base_interface());
		executor.add_node(SegmentedPointcloudCreator->get_node_base_interface());
		executor.add_node(ExtendedOctomapCreator->get_node_base_interface());
		executor.add_node(Pipeline->get_node_base_interface());
		executor.spin();
	});

	// Initialize planner, move group, planning scene and get general info
	MoveIt2APICreator->initPlanner();
	// Initialize visual tools for drawing on rviz
	MoveIt2APICreator->initRvizVisualTools();


    // start the main thread for the grasp pose estimator node to estimate the grasp pose from object coordinates
	std::thread pipeline_thread = std::thread(
		&active_vision_nbv_planning_pipeline::ActiveVisionNbvPlanningPipeline::ActiveVisionNbvPlanningPipelineThread, Pipeline);
	pipeline_thread.detach();

	main_thread->join();
	pipeline_thread.join();

	rclcpp::shutdown();
    return 0;
}