
#pragma once
#ifndef _OCTOMAP_SERVER_HPP_
#define _OCTOMAP_SERVER_HPP_

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rmw/qos_profiles.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/srv/bounding_box_query.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <fruit_picking_octomap/transforms.hpp>
#include <fruit_picking_octomap/conversions.h>

#ifndef COLOR_OCTOMAP_SERVER
#define COLOR_OCTOMAP_SERVER
#include <octomap/ColorOcTree.h>
#endif

#ifdef COLOR_OCTOMAP_SERVER
using PCLPoint = pcl::PointXYZRGB;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
using OcTreeT = octomap::ColorOcTree;
#else
using PCLPoint = pcl::PointXYZ;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
using OcTreeT = octomap::OcTree;
#endif

using OctomapSrv =  octomap_msgs::srv::GetOctomap;
using BBXSrv =  octomap_msgs::srv::BoundingBoxQuery;


namespace ph = std::placeholders;

namespace octomap_server {
    class OctomapServer: public rclcpp::Node {  // we are defining a new node type (class OctomapServer that extend Node class)
        
    protected: // everything can be accessed by the class or by the child classes

        std::shared_ptr<message_filters::Subscriber<
                            sensor_msgs::msg::PointCloud2>> m_pointCloudSub; // pointer to a object of class Subscriber (defined in message_filters namespace) that have as topic type PT2
        std::shared_ptr<tf2_ros::MessageFilter<
                            sensor_msgs::msg::PointCloud2>> m_tfPointCloudSub; // pointer of a object of class message filter (defined in namespace tf2) that have as topic tipe PT2
        
        static std_msgs::msg::ColorRGBA heightMapColor(double h); // declaration of a static function of type ColorRGBA (that is a struct containing 4 floats). It take an height and create a coloration for all the octomap, with color different based on the height

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2
                          >::SharedPtr m_pointCloudPub; // pointer to a object of class Publisher (defined in rclcpp namespace) that have as topic type PT2. The declaration from is the same as the above subscriber
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray
                          >::SharedPtr m_fmarkerPub; // pointer to a object of class Publisher ... 
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray
                          >::SharedPtr m_markerPub; // pointer to a object of class Publisher ... 
        rclcpp::Publisher<octomap_msgs::msg::Octomap
                          >::SharedPtr m_binaryMapPub; // pointer to a object of class Publisher ... 
        rclcpp::Publisher<octomap_msgs::msg::Octomap
                          >::SharedPtr m_fullMapPub; // pointer to a object of class Publisher ... 
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid
                          >::SharedPtr m_mapPub; // pointer to a object of class Publisher ... 
        
        rclcpp::Service<OctomapSrv>::SharedPtr m_octomapBinaryService; 
        rclcpp::Service<OctomapSrv>::SharedPtr m_octomapFullService;
        rclcpp::Service<BBXSrv>::SharedPtr m_clearBBXService;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_resetService;

        std::shared_ptr<tf2_ros::Buffer> buffer_; // buffer to store TF frames
        std::shared_ptr<tf2_ros::TransformListener> m_tfListener; // listered of tf data

        std::shared_ptr<OcTreeT> m_octree; // occupancy octree
        
        octomap::KeyRay m_keyRay;  // temp storage for ray casting
        octomap::OcTreeKey m_updateBBXMin; // a key that is an address of a voxel, it is the minimum value of the bounding box of all known space
        octomap::OcTreeKey m_updateBBXMax; // a key that is an address of a voxel, it is the maximum value of the bounding box of all known space

        double m_maxRange;
        std::string m_worldFrameId; // the map frame
        std::string m_baseFrameId; // base of the robot for ground plane filtering
        bool m_useHeightMap;
        std_msgs::msg::ColorRGBA m_color;
        std_msgs::msg::ColorRGBA m_colorFree;
        double m_colorFactor;
        bool m_publishFreeSpace;
        double m_res;
        unsigned m_treeDepth;
        unsigned m_maxTreeDepth;
        double m_pointcloudMinX;
        double m_pointcloudMaxX;
        double m_pointcloudMinY;
        double m_pointcloudMaxY;
        double m_pointcloudMinZ;
        double m_pointcloudMaxZ;
        double m_occupancyMinZ;
        double m_occupancyMaxZ;
        double m_minSizeX;
        double m_minSizeY;
        bool m_filterSpeckles;
        bool m_filterGroundPlane;
        double m_groundFilterDistance;
        double m_groundFilterAngle;
        double m_groundFilterPlaneDistance;
        bool m_compressMap;

        // downprojected 2D map:
        bool m_incrementalUpdate;
        nav_msgs::msg::OccupancyGrid m_gridmap;
        bool m_publish2DMap;
        bool m_mapOriginChanged;
        octomap::OcTreeKey m_paddedMinKey;
        unsigned m_multires2DScale;
        bool m_projectCompleteMap;
        bool m_useColoredMap;
        
        inline static void updateMinKey(const octomap::OcTreeKey& in,
                                        octomap::OcTreeKey& min) {
            for (unsigned i = 0; i < 3; ++i) {
                min[i] = std::min(in[i], min[i]);
            }
        };

        inline static void updateMaxKey(const octomap::OcTreeKey& in,
                                        octomap::OcTreeKey& max) {
            for (unsigned i = 0; i < 3; ++i) {
                max[i] = std::max(in[i], max[i]);
            }
        };
        
        /// Test if key is within update area of map (2D, ignores height)
        inline bool isInUpdateBBX(const OcTreeT::iterator& it) const {
            // 2^(tree_depth-depth) voxels wide:
            unsigned voxelWidth = (1 << (m_maxTreeDepth - it.getDepth()));
            octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
            return (key[0] + voxelWidth >= m_updateBBXMin[0]
                    && key[1] + voxelWidth >= m_updateBBXMin[1]
                    && key[0] <= m_updateBBXMax[0]
                    && key[1] <= m_updateBBXMax[1]);
        }
        
        inline unsigned mapIdx(int i, int j) const {
            return m_gridmap.info.width * j + i;
        }
        
        inline unsigned mapIdx(const octomap::OcTreeKey& key) const {
            return mapIdx((key[0] - m_paddedMinKey[0]) / m_multires2DScale,
                          (key[1] - m_paddedMinKey[1]) / m_multires2DScale);

        }

        inline bool mapChanged(const nav_msgs::msg::MapMetaData& oldMapInfo,
                               const nav_msgs::msg::MapMetaData& newMapInfo) {
            return (oldMapInfo.height != newMapInfo.height
                    || oldMapInfo.width != newMapInfo.width
                    || oldMapInfo.origin.position.x != newMapInfo.origin.position.x
                    || oldMapInfo.origin.position.y != newMapInfo.origin.position.y);
        }

        void publishBinaryOctoMap(const rclcpp::Time &) const;
        void publishFullOctoMap(const rclcpp::Time &) const;
        virtual void publishAll(const rclcpp::Time &);
        
        virtual void insertScan(
            const geometry_msgs::msg::Vector3  &sensorOrigin,
            const PCLPointCloud& ground,
            const PCLPointCloud& nonground);

        void filterGroundPlane(const PCLPointCloud& pc,
                               PCLPointCloud& ground,
                               PCLPointCloud& nonground) const;

        bool isSpeckleNode(const octomap::OcTreeKey& key) const;

        virtual void handlePreNodeTraversal(const rclcpp::Time &);
        virtual void handlePostNodeTraversal(const rclcpp::Time &);
        virtual void handleNode(const OcTreeT::iterator& it) {};
        virtual void handleNodeInBBX(const OcTreeT::iterator& it) {};
        virtual void handleOccupiedNode(const OcTreeT::iterator& it);
        virtual void handleOccupiedNodeInBBX(const OcTreeT::iterator& it);
        virtual void handleFreeNode(const OcTreeT::iterator& it);
        virtual void handleFreeNodeInBBX(const OcTreeT::iterator& it);
        virtual void update2DMap(const OcTreeT::iterator&, bool);

        virtual void onInit();        
        virtual void subscribe();
        
        void adjustMapData(nav_msgs::msg::OccupancyGrid& map,
                           const nav_msgs::msg::MapMetaData& oldMapInfo) const;
        
    public:        
        explicit OctomapServer(
            const rclcpp::NodeOptions &,
            const std::string = "octomap_server");
        virtual ~OctomapServer();        
        virtual bool octomapBinarySrv(
            const std::shared_ptr<OctomapSrv::Request> ,
            std::shared_ptr<OctomapSrv::Response>);
        virtual bool octomapFullSrv(
            const std::shared_ptr<OctomapSrv::Request> ,
            std::shared_ptr<OctomapSrv::Response>);

        bool clearBBXSrv(
            const std::shared_ptr<BBXSrv::Request>,
            std::shared_ptr<BBXSrv::Response>);
        bool resetSrv(
            const std::shared_ptr<std_srvs::srv::Empty::Request>,
            std::shared_ptr<std_srvs::srv::Empty::Response>);

        virtual void insertCloudCallback(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &);
        virtual bool openFile(const std::string& filename);

    };    
} // octomap_server

#endif  // _OCTOMAP_SERVER_HPP_
