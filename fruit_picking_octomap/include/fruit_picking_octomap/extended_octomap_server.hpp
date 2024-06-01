#pragma once
#ifndef _EXTENDED_OCTOMAP_SERVER_HPP_
#define _EXTENDED_OCTOMAP_SERVER_HPP_


#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rmw/qos_profiles.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
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

#include <eigen3/Eigen/Eigen>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/srv/bounding_box_query.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <fruit_picking_octomap/extended_octomap_data.hpp>
#include "fruit_picking_interfaces/msg/pointcloud_array.hpp"
#include <std_srvs/srv/set_bool.hpp>
#include "message_filters/sync_policies/approximate_time.h"

#ifndef COLOR_OCTOMAP_SERVER
// #define COLOR_OCTOMAP_SERVER
#include <octomap/ColorOcTree.h>
#endif



// Definitions

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

using ExtendedOctomapData = extended_octomap_data::ExtendedOctomapData;

using SyncPolicyArray = message_filters::sync_policies::ApproximateTime<fruit_picking_interfaces::msg::PointcloudArray, geometry_msgs::msg::TransformStamped>;
using SynchronizerArray= message_filters::Synchronizer<SyncPolicyArray>;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, geometry_msgs::msg::TransformStamped>;
using Synchronizer= message_filters::Synchronizer<SyncPolicy>;

typedef octomap::unordered_ns::unordered_map<octomap::OcTreeKey, ExtendedOctomapData, octomap::OcTreeKey::KeyHash> ExtendedOctomapMap;

typedef octomap::unordered_ns::unordered_map<octomap::OcTreeKey, std::list<std::tuple<int, std::string, int, float>>, octomap::OcTreeKey::KeyHash> CollisionOcTreeKeys;

namespace ph = std::placeholders;


namespace extended_octomap_server{

    
    class ExtendedOctomapServer: public rclcpp::Node {

    protected:

        // Octomap variables

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
        bool m_useColoredMap;
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

        // new value for the message filter queue
        int messageFilterQueue;

        // new bool for binary and full octomap, centers pointcloud and 2d map
        bool publishFreeCells;
        bool publishOctomapBinary;
        bool publishOctomapFull;
        bool publishCentersPointcloud;
        bool publish2DProjectedMap;

        bool partialPointcloudSubscription; // tells to use not the full pointcloud, but a partial

    



        // Extended Octomap Variables

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



        // Octomap Methods

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
        // virtual void handleNode(const OcTreeT::iterator& it) {};
        // virtual void handleNodeInBBX(const OcTreeT::iterator& it) {};
        virtual void handleOccupiedNode(const OcTreeT::iterator& it);
        virtual void handleOccupiedNodeInBBX(const OcTreeT::iterator& it);
        virtual void handleFreeNode(const OcTreeT::iterator& it);
        virtual void handleFreeNodeInBBX(const OcTreeT::iterator& it);
        virtual void update2DMap(const OcTreeT::iterator&, bool);

        virtual void createPubSub();        
        
        void adjustMapData(nav_msgs::msg::OccupancyGrid& map,
                           const nav_msgs::msg::MapMetaData& oldMapInfo) const;




        // Extended Octomap Methods


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





        // Transforms
        /** \brief Obtain the transformation matrix from TF into an Eigen form
        * \param bt the TF transformation
        * \param out_mat the Eigen transformation
        */
        void transformAsMatrix(
            const tf2::Transform &, Eigen::Matrix4f &);
        Eigen::Matrix4f transformAsMatrix(
            const geometry_msgs::msg::TransformStamped &);





        
        // Conversions
        /**
         * @brief Conversion from octomap::point3d_list
         (e.g. all occupied nodes from getOccupied()) to
        * sensor_msgs::msg::PointCloud2
        *
        * @param points
        * @param cloud
        */
        void pointsOctomapToPointCloud2(const octomap::point3d_list& points,
                                        sensor_msgs::msg::PointCloud2& cloud);

        /**
         * @brief Conversion from a sensor_msgs::msg::PointCLoud2 to
         octomap::Pointcloud, used internally in OctoMap     
        *
        * @param cloud
        * @param octomapCloud
        */

        void pointCloud2ToOctomap(const sensor_msgs::msg::PointCloud2& cloud,
                                octomap::Pointcloud& octomapCloud);

        /// Conversion from octomap::point3d to geometry_msgs::msg::Point
        static inline geometry_msgs::msg::Point pointOctomapToMsg(
            const octomap::point3d& octomapPt){
            geometry_msgs::msg::Point pt;
            pt.x = octomapPt.x();
            pt.y = octomapPt.y();
            pt.z = octomapPt.z();

            return pt;
        }

        /// Conversion from geometry_msgs::msg::Point to octomap::point3d
        static inline octomap::point3d pointMsgToOctomap(
            const geometry_msgs::msg::Point& ptMsg){
            return octomap::point3d(ptMsg.x, ptMsg.y, ptMsg.z);
        }

        /// Conversion from octomap::point3d to tf2::Point
        static inline geometry_msgs::msg::Point pointOctomapToTf(
            const octomap::point3d &octomapPt) {
            geometry_msgs::msg::Point pt;
            pt.x = octomapPt.x();
            pt.y = octomapPt.y();
            pt.z = octomapPt.z();
            return pt;        
        }

        /// Conversion from tf2::Point to octomap::point3d
        static inline octomap::point3d pointTfToOctomap(
            const geometry_msgs::msg::Point& ptTf){
            return octomap::point3d(ptTf.x, ptTf.y, ptTf.z);
        }    
        
        static inline octomap::point3d pointTfToOctomap(
            const geometry_msgs::msg::Vector3& ptTf){
            return octomap::point3d(ptTf.x, ptTf.y, ptTf.z);
        }
        
        /// Conversion from octomap Quaternion to tf2::Quaternion
        static inline tf2::Quaternion quaternionOctomapToTf(
            const octomath::Quaternion& octomapQ){
            return tf2::Quaternion(
                octomapQ.x(), octomapQ.y(), octomapQ.z(), octomapQ.u());
        }
        
        /// Conversion from tf2::Quaternion to octomap Quaternion
        static inline octomath::Quaternion quaternionTfToOctomap(
            const tf2::Quaternion& qTf){
            return octomath::Quaternion(qTf.w(), qTf.x(), qTf.y(), qTf.z());
        }

        static inline octomath::Quaternion quaternionTfToOctomap(
            const geometry_msgs::msg::Quaternion &qTf){
            return octomath::Quaternion(qTf.w, qTf.x, qTf.y, qTf.z);
        }
        
        /// Conversion from octomap::pose6f to tf2::Pose
        static inline geometry_msgs::msg::Pose poseOctomapToTf(
            const octomap::pose6d& octomapPose){
            auto r = quaternionOctomapToTf(octomapPose.rot());
            geometry_msgs::msg::Quaternion orientation;
            orientation.x = r.x();
            orientation.y = r.y();
            orientation.z = r.z();
            orientation.w = r.w();
            
            geometry_msgs::msg::Pose pose;
            pose.position = pointOctomapToTf(octomapPose.trans());        
            pose.orientation = orientation;        
            return pose;     
        }
        
        /// Conversion from tf2::Pose to octomap::pose6d
        static inline octomap::pose6d poseTfToOctomap(
            const geometry_msgs::msg::Pose& poseTf) {
            return octomap::pose6d(pointTfToOctomap(poseTf.position),
                                quaternionTfToOctomap(poseTf.orientation));
        }
        

    public:

        // Octomap Server Methods     
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


        // Extended Octomap Server Methods
        explicit ExtendedOctomapServer(
            const rclcpp::NodeOptions &,
            const std::string = "extended_octomap_server");

        virtual ~ExtendedOctomapServer();
        
        // Callback to insert a segmented pointcloud (for complexity reasons), using as tf the one obtained from a /partial_tf topic coming from the 
        // segmentation node
        virtual void insertPartialCloudCallback(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &,
            const geometry_msgs::msg::TransformStamped::ConstSharedPtr &
            );
        
    };

} // end namespace extended_octomap
#endif