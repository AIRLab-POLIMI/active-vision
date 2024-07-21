#pragma once
#ifndef _ACTIVE_VISION_PREDEFINED_PLANNING_PIPELINE_HPP_
#define _ACTIVE_VISION_PREDEFINED_PLANNING_PIPELINE_HPP_

#include <fruit_picking_planning/active_vision_pipeline.hpp>

namespace active_vision_predefined_planning_pipeline {

class ActiveVisionPredefinedPlanningPipeline : public active_vision_pipeline::ActiveVisionPipeline {
    
    protected:

        // Variable containing the predefined planning waypoints
        std::string predefinedPlanning_;
        std::vector<std::array<double, 6>> PlanningPoses_;
        std::vector<Eigen::Isometry3d> CartesianPlanningPoses_;

        void createDataSub();

        void saveData(
            const Image::ConstSharedPtr& rgb_msg,
            const Image::ConstSharedPtr& depth_msg,
            const CameraInfo::ConstSharedPtr& camera_info_msg);

        // Function that creates a vector of planned positions
        std::vector<std::array<double, 6>> createPlanningPoses();

        double reconstructionMetric(bool visualization);

    
    
    public:

        ActiveVisionPredefinedPlanningPipeline(
        std::shared_ptr<MoveIt2APIs> MoveIt2API_creator,
        std::shared_ptr<full_pointcloud::FullPointcloud> pointcloud_creator,
        std::shared_ptr<segmented_pointcloud::SegmentedPointcloud> segmented_pointcloud_creator,
        std::shared_ptr<extended_octomap_server::ExtendedOctomapServer> extended_octomap_creator,
        std::shared_ptr<rclcpp::Node> segmentationClientNode,
        const rclcpp::NodeOptions &options,
        const std::string name = "active_vision_predefined_planning_pipeline");

        void ActiveVisionPredefinedPlanningPipelineThread();

    };

}
#endif