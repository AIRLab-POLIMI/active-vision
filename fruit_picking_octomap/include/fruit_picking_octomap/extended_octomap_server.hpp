#pragma once
#ifndef _EXTENDED_OCTOMAP_SERVER_HPP_
#define _EXTENDED_OCTOMAP_SERVER_HPP_

#include <fruit_picking_octomap/octomap_server.hpp>
#include <fruit_picking_octomap/extended_octomap_data.hpp>
#include "fruit_picking_interfaces/msg/pointcloud_array.hpp"
#include <std_srvs/srv/set_bool.hpp>
#include "message_filters/sync_policies/approximate_time.h"


namespace extended_octomap_server{


    // Definitions

    using ExtendedOctomapData = extended_octomap_data::ExtendedOctomapData;

    using SyncPolicyArray = message_filters::sync_policies::ApproximateTime<fruit_picking_interfaces::msg::PointcloudArray, geometry_msgs::msg::TransformStamped>;
    using SynchronizerArray= message_filters::Synchronizer<SyncPolicyArray>;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, geometry_msgs::msg::TransformStamped>;
    using Synchronizer= message_filters::Synchronizer<SyncPolicy>;

    typedef octomap::unordered_ns::unordered_map<octomap::OcTreeKey, ExtendedOctomapData, octomap::OcTreeKey::KeyHash> ExtendedOctomapMap;

    typedef octomap::unordered_ns::unordered_map<octomap::OcTreeKey, std::list<std::tuple<int, std::string, int, float>>, octomap::OcTreeKey::KeyHash> CollisionOcTreeKeys;

    
    // Main class

    class ExtendedOctomapServer : public octomap_server::OctomapServer{

    protected:

        // Variables

        // Subscriber related to the segmented pointcloud (pointcloud containing all the segmented instances)
        std::shared_ptr<message_filters::Subscriber<
                            sensor_msgs::msg::PointCloud2>> segmentedPointcloudSub;
        // Subscriber related to the array of segmented pointclouds (each pointcloud refers to a single instance)
        std::shared_ptr<message_filters::Subscriber<
                            fruit_picking_interfaces::msg::PointcloudArray>> segmentedPointcloudsArraySub;
        // Subscriber for the tf related to the segmented pointcloud (tf related to the segmented pointcloud used in the segmented pointcloud callbacks)
        std::shared_ptr<message_filters::Subscriber<
                            geometry_msgs::msg::TransformStamped>> segmentedPointcloudTfSub;
        // Subscriber for the tf related to the segmented pointclouds array (tf related to the segmented pointcloud used in the segmented pointclouds array callbacks)
        std::shared_ptr<message_filters::Subscriber<
                            geometry_msgs::msg::TransformStamped>> segmentedPointcloudsArrayTfSub;
        // Subscriber for the tf related to the partial pointcloud (tf related to the segmented pointcloud used in the insert cloud callback
        // The segmented pointcloud is used instead of the full pointcloud to ease the computation of the octomap)
        std::shared_ptr<message_filters::Subscriber<
                            geometry_msgs::msg::TransformStamped>> partialTfSub;
        
        // Synchronizer for the segmented pointclouds array and segmented tf
        std::shared_ptr<SynchronizerArray> sync_segmented_pointclouds_array_;
        // Synchronizer for the segmented pointcloud and segmented tf
        std::shared_ptr<Synchronizer> sync_segmented_pointcloud_;
        // Synchronizer for the partial pointcloud and partial tf
        std::shared_ptr<Synchronizer> sync_partial_pointcloud_;


        rclcpp::Publisher<visualization_msgs::msg::MarkerArray
                          >::SharedPtr confidenceMarkerPub;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray
                          >::SharedPtr semanticClassMarkerPub;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray
                          >::SharedPtr instancesMarkerPub;

        std::shared_ptr<ExtendedOctomapMap> extended_octomap_map;

        octomap::KeySet global_free_cells, global_occupied_cells;

        bool processFreeSpace; // tells if saving free space on a data structure is required
        bool publishConfidence; // tells if the confidence info isnide the extended octomap map and markers are required
        bool publishSemantic; // tells if the semantic classes inside the extended map and markers are required
        bool publishInstances; // tells if the instances markers are required
        bool segmentedPointcloudsArraySubscription; // tells if the node needs to subscribe to the segmented pointclouds array topic
        bool segmentedPointcloudSubscription; // tells if the node needs to subscribe to the segmented pointcloud topic


        // Bool that are used to activate or deactivate the insert semantic and insert cloud callbacks. They are set using some services
        bool insertCloudActive;
        bool insertSegmentedActive;

        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr insertCloudActiveService_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr insertSegmentedActiveService_;

        // Map to save octreekeys that contains collision points between different instances
        std::shared_ptr<CollisionOcTreeKeys> collisionKeys;

        // Variable to keep track of the instances
        int currentMaxInstance;

        // Map to keep track of the coloration (hue value) of each semantic class (for visualization purposes)
        std::map<std::string, float> colorMap;


        // Methods

        virtual void onInit();   

        // If the subscription to the pointcloud need to obtain a partial (or segmented) pointcloud
        virtual void subscribe();
    

        virtual void insertScan(
            const geometry_msgs::msg::Vector3  &sensorOrigin,
            const PCLPointCloud& ground,
            const PCLPointCloud& nonground);

        virtual void insertSegmentedPointcloudCallback(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &segmented_pointcloud,
            const geometry_msgs::msg::TransformStamped::ConstSharedPtr &segmented_tf);

        virtual void insertSegmentedPointcloudsArrayCallback(
            const fruit_picking_interfaces::msg::PointcloudArray::ConstSharedPtr &segmented_pointclouds_array, 
            const geometry_msgs::msg::TransformStamped::ConstSharedPtr &segmented_tf);

        void publishConfidenceMarkers(const rclcpp::Time &) const;

        void publishSemanticClassMarkers(const rclcpp::Time &) const;

        void publishInstancesMarkers(const rclcpp::Time &) const;

        void setInsertCloudActive(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response);

        void setinsertSegmentedActive(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response);

        // Function to find the most frequent instance of a set of octreekeys
        int findMostFrequentInstance(ExtendedOctomapMap& map, const octomap::KeySet& pointcloudKeys);

        // Function to count the number of points inside a octree node
        int countPointsInVoxel(const PCLPointCloud& pointcloud, const octomap::OcTreeKey& targetKey, const std::shared_ptr<OcTreeT> m_octree);
        

    public:

        explicit ExtendedOctomapServer(
            const rclcpp::NodeOptions &options,
            const std::string node_name = "extended_octomap_server");

        virtual ~ExtendedOctomapServer();

        // Callback to insert the full pointcloud, using as tf the one obtained from the tf listerer connected directly to the pointcloud subscriber
        virtual void insertCloudCallback(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &);
        
        // Callback to insert a segmented pointcloud (for complexity reasons), using as tf the one obtained from a /partial_tf topic coming from the 
        // segmentation node
        virtual void insertPartialCloudCallback(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &,
            const geometry_msgs::msg::TransformStamped::ConstSharedPtr &
            );
        
    };

} // end namespace extended_octomap
#endif