#pragma once
#ifndef _ACTIVE_VISION_NBV_PLANNING_PIPELINE_HPP_
#define _ACTIVE_VISION_NBV_PLANNING_PIPELINE_HPP_

#include <fruit_picking_planning/active_vision_pipeline.hpp>
#include <random>
#include <omp.h>

namespace active_vision_nbv_planning_pipeline {

class ActiveVisionNbvPlanningPipeline : public active_vision_pipeline::ActiveVisionPipeline {
    
    protected:

        int orientations_;
        int candidateViewpointsNumber_;
        std::string planeTypeCandidateViewpoints_;
        float movementRange_;

        std::array<double, 6> initialPosition_;
        Eigen::Isometry3d initialPositionCartesian_;
        std::vector<Eigen::Isometry3d> candidateViewpoints_;
        Eigen::Isometry3d NBV_pose_;
        geometry_msgs::msg::PoseStamped::SharedPtr NBV_pose_ptr_;
        int maxNBVPlanningSteps_;

        std::shared_ptr<ExtendedOctomapMap> extendedOctomapMap_;
        std::shared_ptr<OcTreeT> octree_;

        Eigen::Vector3d octomapCenter_ = Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d furthestPoint_ = Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d closestPoint_ = Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d highestPoint_ = Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d lowestPoint_ = Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d leftMostPoint_ = Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d rightMostPoint_ = Eigen::Vector3d(0, 0, 0);
        double newClosest_ = 0.0, newFurthest_ = 0.0, newLowest_ = 0.0, newHighest_ = 0.0, newLeftMost_ = 0.0, newRightMost_ = 0.0;

        float centralAttentionFrontDistanceRatio_;
        float centralAttentionWidthDistanceRatio_;
        float centralAttentionHeightDistanceRatio_;


        double maxRayDepth_;
        double rayStepProportion_;
        std::string rayCastingType_;
        bool rayCastingVis_;
        std::string utilityType_;
        bool utilityVis_;

        // This variable is set to true whenever some semantic is found. In this case, some entropy will be calculated and
        // if some next step will have no confidence updated, the same pose will be explored
        // If this variable is false, if during the step no confidence is updated, the next pose will be not the previous one 
        // but a random one
        bool semanticFound_ = false;


        void createDataSub();

        void saveData(
            const Image::ConstSharedPtr& rgb_msg,
            const Image::ConstSharedPtr& depth_msg,
            const CameraInfo::ConstSharedPtr& camera_info_msg);

        std::array<double, 6> getInitialPosition();

        std::vector<Eigen::Isometry3d> generatePlaneCandidateViewpoints(
            const Eigen::Isometry3d referencePose, 
            int orientations,
            std::string planeTypeCandidateViewpoints_, 
            int N, 
            float sideLength);

        geometry_msgs::msg::Pose findUpperCenterPose(const std::vector<Eigen::Isometry3d>& poses);

        geometry_msgs::msg::PoseStamped::SharedPtr eigenIsometry3dToPoseStamped(const Eigen::Isometry3d& isometry);

        void visualizeArrowPose(const Eigen::Isometry3d& pose, double length, rviz_visual_tools::Colors color, rviz_visual_tools::Scales scale);

        Eigen::Isometry3d chooseNBVRandom(const std::vector<Eigen::Isometry3d>& poses);

        void visualizeFrustum(const Eigen::Isometry3d& starting_pose, double fov_w_deg, double fov_h_deg, double frustum_depth);

        void visualizeFrustumBase(const Eigen::Isometry3d& starting_pose, double fov_w_deg, double fov_h_deg, double frustum_depth);

        std::vector<Eigen::Vector3d> generateFrustumBaseGrid(const Eigen::Isometry3d& starting_pose, double fov_w_deg, double fov_h_deg, double resolution_m, double frustum_depth);

        std::vector<Eigen::Vector3d> generateFrustumBaseDirections(const Eigen::Isometry3d& starting_pose, double fov_w_deg, double fov_h_deg, double resolution_m);

        Eigen::Vector3d getOccupancyMapCenter();

        Eigen::Vector3d findFurthestPoint(const Eigen::Isometry3d& current_pose);

        Eigen::Vector3d findClosestPoint(const Eigen::Isometry3d& current_pose);

        Eigen::Vector3d findHighestPoint(const Eigen::Isometry3d& current_pose);

        Eigen::Vector3d findLowestPoint(const Eigen::Isometry3d& current_pose);

        Eigen::Vector3d findLeftMostPoint(const Eigen::Isometry3d& current_pose);

        Eigen::Vector3d findRightMostPoint(const Eigen::Isometry3d& current_pose);

        std::vector<Eigen::Vector3d> getSemanticArea(bool visualization);

        std::vector<Eigen::Vector3d> generateFrustumBaseAttentionDirections(const Eigen::Isometry3d& starting_pose, double fov_w_deg, double fov_h_deg, bool visualization);
        
        octomap::KeySet performNaiveRayCasting(Eigen::Isometry3d pose, double fov_w, double fov_h, bool visualization);

        octomap::KeySet performRayCasting(Eigen::Isometry3d pose, double fov_w, double fov_h);
    
        octomap::KeySet performRayCastingAttention(Eigen::Isometry3d pose, double fov_w, double fov_h, bool visualization);

        float utilityCalculation(octomap::KeySet voxels, std::string utility_type);

        Eigen::Isometry3d chooseNBV(const std::vector<Eigen::Isometry3d>& poses, Eigen::Isometry3d current_pose);

        double reconstructionMetric(bool visualization);

    
    
    public:

        ActiveVisionNbvPlanningPipeline(
            std::shared_ptr<MoveIt2APIs> MoveIt2API_creator,
            std::shared_ptr<full_pointcloud::FullPointcloud> pointcloud_creator,
            std::shared_ptr<segmented_pointcloud::SegmentedPointcloud> segmented_pointcloud_creator,
            std::shared_ptr<extended_octomap_server::ExtendedOctomapServer> extended_octomap_creator,
            std::shared_ptr<rclcpp::Node> segmentationClientNode,
            const rclcpp::NodeOptions &options,
            const std::string name = "active_vision_nbv_planning_pipeline");

        void ActiveVisionNbvPlanningPipelineThread();

    };

}
#endif