#pragma once
#ifndef _EXTENDED_OCTOMAP_SERVER_HPP_
#define _EXTENDED_OCTOMAP_SERVER_HPP_

#include <fruit_picking_octomap/octomap_server.hpp>
#include "fruit_picking_interfaces/msg/pointcloud_array.hpp"
#include <std_srvs/srv/set_bool.hpp>


namespace extended_octomap_server{


    class ExtendedOctomapData{
    
    public:

        // Constructor used to initialize the voxel extended data. Called by the insertCloud callback
        ExtendedOctomapData();

        void setSemanticClassNoColor(std::string semantic_class);

        void setSemanticClass(std::string semantic_class);

        void setConfidenceNoColor(float confidence);

        void setConfidence(float confidence);

        void setConfidenceMaxFusion(std::string semantic_class, float confidence, float penalization);

        void setInstanceNoColor(int instance);

        void setInstance(int instance);

        void setSemanticColor(std::string semantic_class);

        void setDirectConfidenceColor(float confidence);

        void setHeatConfidenceColor(float confidence);

        void setInstanceColor(int instance);

        std::string getSemanticClass();

        float getConfidence();

        int getInstance();

        float semantic_r, semantic_g, semantic_b, semantic_a;
        float confidence_r, confidence_g, confidence_b, confidence_a;
        float instance_r, instance_g, instance_b, instance_a;


    protected:

        float confidence;
        std::string semantic_class;
        int instance;
    };



    typedef octomap::unordered_ns::unordered_map<octomap::OcTreeKey, ExtendedOctomapData, octomap::OcTreeKey::KeyHash> ExtendedOctomapMap;

    
    class ExtendedOctomapServer : public octomap_server::OctomapServer{

    protected:

        // Variables

        std::shared_ptr<message_filters::Subscriber<
                            sensor_msgs::msg::PointCloud2>> segmentedPointcloudSub;
        std::shared_ptr<tf2_ros::MessageFilter<
                            sensor_msgs::msg::PointCloud2>> tfSegmentedPointcloudSub;
        std::shared_ptr<message_filters::Subscriber<
                            fruit_picking_interfaces::msg::PointcloudArray>> segmentedPointcloudsArraySub;
        std::shared_ptr<tf2_ros::MessageFilter<
                            fruit_picking_interfaces::msg::PointcloudArray>> tfSegmentedPointcloudsArraySub;

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
        bool semanticPointcloudSubscription; // tells if the node needs to subscribe to the semantic pointcloud topic
        bool semanticPointcloudsArraySubscription; // tells if the node needs to subscribe to the semantic pointclouds array topic

        // Bool that are used to activate or deactivate the callbacks. They are set using some services
        bool insertCloudActive;
        bool insertSemanticActive;

        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr insertCloudActiveService_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr insertSemanticActiveService_;

        // Variable to keep track of the instances
        int currentMaxInstance;


        // Methods

        virtual void onInit();       

        virtual void insertScan(
            const geometry_msgs::msg::Vector3  &sensorOrigin,
            const PCLPointCloud& ground,
            const PCLPointCloud& nonground);

        virtual void insertSemanticCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &reduced_cloud);

        virtual void insertSemanticArrayCallback(const fruit_picking_interfaces::msg::PointcloudArray::ConstSharedPtr &reduced_cloud_array);

        void publishConfidenceMarkers(const rclcpp::Time &) const;

        void publishSemanticClassMarkers(const rclcpp::Time &) const;

        void publishInstancesMarkers(const rclcpp::Time &) const;

        void setInsertCloudActive(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response);

        void setInsertSemanticActive(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response);
        

    public:

        explicit ExtendedOctomapServer(
            const rclcpp::NodeOptions &options,
            const std::string node_name = "extended_octomap_server");

        virtual ~ExtendedOctomapServer();

        virtual void insertCloudCallback(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &);
        
    };

} // end namespace extended_octomap
#endif