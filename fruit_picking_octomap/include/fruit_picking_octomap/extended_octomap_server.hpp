#pragma once
#ifndef _EXTENDED_OCTOMAP_SERVER_HPP_
#define _EXTENDED_OCTOMAP_SERVER_HPP_

#include <fruit_picking_octomap/octomap_server.hpp>

namespace extended_octomap_server{

    
    class ExtendedOctomapData{
    
    public:

        ExtendedOctomapData(){}
        
        ExtendedOctomapData(float confidence, std::string semantic_class){
            this->confidence = confidence;
            this->semantic_class = semantic_class;
        }

        float confidence;
        std::string semantic_class;
    };



    typedef octomap::unordered_ns::unordered_map<octomap::OcTreeKey, ExtendedOctomapData, octomap::OcTreeKey::KeyHash> ExtendedOctomapMap;

    
    class ExtendedOctomapServer : public octomap_server::OctomapServer{

    protected:

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray
                          >::SharedPtr confidenceMarkerPub;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray
                          >::SharedPtr semanticClassMarkerPub;

        std::shared_ptr<ExtendedOctomapMap> extended_octomap_map;

        octomap::KeySet global_free_cells, global_occupied_cells;

        bool m_processFreeSpace;


        virtual void onInit();        


        virtual void insertScan(
            const geometry_msgs::msg::Vector3  &sensorOrigin,
            const PCLPointCloud& ground,
            const PCLPointCloud& nonground);

        virtual void insertSemanticCallback();

        void publishConfidenceMarkers(const rclcpp::Time &) const;

        void publishSemanticClassMarkers(const rclcpp::Time &) const;


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