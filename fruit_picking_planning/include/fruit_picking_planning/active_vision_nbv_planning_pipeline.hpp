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

        double maxRayDepth_;
        double rayStepProportion_;
        std::string rayCastingType_;
        bool rayCastingVis_;


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

        std::vector<Eigen::Vector3d> getSemanticArea(bool visualization);

        std::vector<Eigen::Vector3d> generateFrustumBaseAttentionDirections(const Eigen::Isometry3d& starting_pose, double fov_w_deg, double fov_h_deg, bool visualization);
        
        octomap::KeySet performNaiveRayCasting(Eigen::Isometry3d pose, double fov_w, double fov_h, bool visualization);

        octomap::KeySet performRayCasting(Eigen::Isometry3d pose, double fov_w, double fov_h);
    
        octomap::KeySet performRayCastingAttention(Eigen::Isometry3d pose, double fov_w, double fov_h, bool visualization);

        Eigen::Isometry3d chooseNBV(const std::vector<Eigen::Isometry3d>& poses);


    
    
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