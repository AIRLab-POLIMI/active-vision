#pragma once
#ifndef _EXTENDED_OCTOMAP_SERVER_HPP_
#define _EXTENDED_OCTOMAP_SERVER_HPP_

#include <fruit_picking_octomap/octomap_server.hpp>

namespace extended_octomap_server{

    enum semantic{
        none,
        tomato
    };
    
    class ExtendedOctomapData{
    
    public:

        ExtendedOctomapData(){
            this->confidence = 0.0;
            this->semantic_class = none;
            setColor();
        }

        ExtendedOctomapData(semantic semantic_class){
            this->semantic_class = semantic_class;
            setColor();
        }
        
        ExtendedOctomapData(float confidence, semantic semantic_class){
            this->confidence = confidence;
            this->semantic_class = semantic_class;
            setColor();
        }

        void setColor(){
            if (semantic_class == tomato){
                r = 1.0;
                g = 0.0;
                b = 0.0;
            }
            else if (semantic_class == none){
                r = 1.0;
                g = 1.0;
                b = 1.0;
            }
            a = 1.0;
        }

        float confidence;
        semantic semantic_class;
        float r, g, b, a;
    };



    typedef octomap::unordered_ns::unordered_map<octomap::OcTreeKey, ExtendedOctomapData, octomap::OcTreeKey::KeyHash> ExtendedOctomapMap;

    
    class ExtendedOctomapServer : public octomap_server::OctomapServer{

    protected:

        std::shared_ptr<message_filters::Subscriber<
                            sensor_msgs::msg::PointCloud2>> m_reducedPointCloudSub;
        std::shared_ptr<tf2_ros::MessageFilter<
                            sensor_msgs::msg::PointCloud2>> m_tfReducedPointCloudSub;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray
                          >::SharedPtr confidenceMarkerPub;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray
                          >::SharedPtr semanticClassMarkerPub;

        std::shared_ptr<ExtendedOctomapMap> extended_octomap_map;

        octomap::KeySet global_free_cells, global_occupied_cells;

        bool m_processFreeSpace;
        bool publishConfidence;
        bool publishSemantic;

        virtual void onInit();        


        virtual void insertScan(
            const geometry_msgs::msg::Vector3  &sensorOrigin,
            const PCLPointCloud& ground,
            const PCLPointCloud& nonground);

        virtual void insertSemanticCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &reduced_cloud);

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