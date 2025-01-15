#include <av_octomap/extended_octomap_server.hpp>

namespace extended_octomap_server {

    /**
     * @brief Constructor for the ExtendedOctomapServer class.
     * 
     * This constructor initializes the ExtendedOctomapServer class, setting up various parameters and configurations
     * required for managing and processing OctoMaps with extended data. It declares ROS 2 parameters, initializes
     * the OctoMap object, and sets up the tf2 buffer and listener for transformation handling.
     * 
     * Key initializations include:
     * - Setting default values for parameters such as map resolution, color settings, and filtering options.
     * - Declaring ROS 2 parameters for configuration.
     * - Initializing the OctoMap object with specified resolution and probability parameters.
     * - Setting up the tf2 buffer and listener for transformation handling.
     * - Configuring various settings such as point cloud boundaries, ground filtering, and map compression.
     * 
     * @param options Node options for ROS 2 node initialization.
     * @param node_name Name of the ROS 2 node.
     */
    ExtendedOctomapServer::ExtendedOctomapServer(const rclcpp::NodeOptions &options, const std::string node_name):
        Node(node_name, options),
        m_octree(NULL),
        m_maxRange(20),
        m_worldFrameId("/map"),
        m_baseFrameId("base_footprint"),
        m_useHeightMap(true),
        m_colorFactor(0.8),
        m_useColoredMap(false),
        m_publishFreeSpace(false),
        m_res(0.05),
        m_treeDepth(0),
        m_maxTreeDepth(0),
        m_pointcloudMinX(-std::numeric_limits<double>::max()),
        m_pointcloudMaxX(std::numeric_limits<double>::max()),
        m_pointcloudMinY(-std::numeric_limits<double>::max()),
        m_pointcloudMaxY(std::numeric_limits<double>::max()),
        m_pointcloudMinZ(-std::numeric_limits<double>::max()),
        m_pointcloudMaxZ(std::numeric_limits<double>::max()),
        m_occupancyMinZ(-std::numeric_limits<double>::max()),
        m_occupancyMaxZ(std::numeric_limits<double>::max()),
        m_minSizeX(0.0),
        m_minSizeY(0.0),
        m_filterSpeckles(false),
        m_filterGroundPlane(false),
        m_groundFilterDistance(0.04),
        m_groundFilterAngle(0.15),
        m_groundFilterPlaneDistance(0.07),
        m_compressMap(true),
        m_incrementalUpdate(false),
         // new value for the message filter queue
        messageFilterQueue(5),

        // new bool for binary and full octomap, centers pointcloud and 2d map
        publishFreeCells(false),
        publishOctomapBinary(false),
        publishOctomapFull(false),
        publishCentersPointcloud(false),
        publish2DProjectedMap(false),

        search_neighboorhood_ray(5),
        correction_neighboorhood_ray(5),
        outlier_detection(0.2)

    
    
    {
        RCLCPP_INFO(this->get_logger(), "Octomap server's constructor started.");

        rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME); // another structure to declare and define a shared pointer of type Clock
        auto cache_time = std::chrono::minutes(1); // For example, setting it to 10 minutes
        this->buffer_ = std::make_shared<tf2_ros::Buffer>(clock, tf2::durationFromSec(cache_time.count() * 60.0));
        this->buffer_->setUsingDedicatedThread(true); // tells the buffer that multiple thread will serve it
        this->m_tfListener = std::make_shared<tf2_ros::TransformListener>(
            *buffer_, this, false); // this refers to the class OctomapServer
        
        m_worldFrameId = this->declare_parameter("frame_id", m_worldFrameId);
        m_baseFrameId = this->declare_parameter("base_frame_id", m_baseFrameId);
        m_useHeightMap = this->declare_parameter("height_map", m_useHeightMap);
        m_useColoredMap = this->declare_parameter("colored_map", m_useColoredMap);
        m_colorFactor = this->declare_parameter("color_factor", m_colorFactor);

        m_pointcloudMinX = this->declare_parameter(
            "pointcloud_min_x", m_pointcloudMinX);
        m_pointcloudMaxX = this->declare_parameter(
            "pointcloud_max_x", m_pointcloudMaxX);
        m_pointcloudMinY = this->declare_parameter(
            "pointcloud_min_y", m_pointcloudMinY);
        m_pointcloudMaxY = this->declare_parameter(
            "pointcloud_max_y", m_pointcloudMaxY);
        m_pointcloudMinZ = this->declare_parameter(
            "pointcloud_min_z", m_pointcloudMinZ);
        m_pointcloudMaxZ = this->declare_parameter(
            "pointcloud_max_z", m_pointcloudMaxZ);
        m_occupancyMinZ = this->declare_parameter(
            "occupancy_min_z", m_occupancyMinZ);
        m_occupancyMaxZ = this->declare_parameter(
            "occupancy_max_z", m_occupancyMaxZ);
        m_minSizeX = this->declare_parameter(
            "min_x_size", m_minSizeX);
        m_minSizeY = this->declare_parameter(
            "min_y_size", m_minSizeY);

        m_filterSpeckles = this->declare_parameter(
            "filter_speckles", m_filterSpeckles);
        m_filterGroundPlane = this->declare_parameter(
            "filter_ground", m_filterGroundPlane);
        // distance of points from plane for RANSAC
        m_groundFilterDistance = this->declare_parameter(
            "ground_filter/distance", m_groundFilterDistance);
        // angular derivation of found plane:
        m_groundFilterAngle = this->declare_parameter(
            "ground_filter/angle", m_groundFilterAngle);
        m_groundFilterPlaneDistance = this->declare_parameter(
            "ground_filter/plane_distance", m_groundFilterPlaneDistance);

        m_maxRange = this->declare_parameter(
            "sensor_model/max_range", m_maxRange);

        m_res = this->declare_parameter("resolution", m_res);
        double probHit = this->declare_parameter("sensor_model/hit", 0.7);
        double probMiss = this->declare_parameter("sensor_model/miss", 0.4);
        double thresMin = this->declare_parameter("sensor_model/min", 0.12);
        double thresMax = this->declare_parameter("sensor_model/max", 0.97);
        m_compressMap = this->declare_parameter("compress_map", m_compressMap);
        m_incrementalUpdate = this->declare_parameter(
            "incremental_2D_projection", false);

        if (m_filterGroundPlane &&
            (m_pointcloudMinZ > 0.0 || m_pointcloudMaxZ < 0.0)) {
            std::string msg = "You enabled ground filtering but incoming pointclouds " +
                std::string("will be pre-filtered in [%ld, %ld], excluding the") +
                std::string("ground level z=0 This will not work.");
            RCLCPP_WARN(this->get_logger(), msg.c_str(), m_pointcloudMinZ, m_pointcloudMaxZ);
        }

        if (m_useHeightMap && m_useColoredMap) {
            std::string msg = std::string("You enabled both height map and RGB") +
                "color registration. This is contradictory. Defaulting to height map."; 
            RCLCPP_WARN(this->get_logger(), msg.c_str());
            m_useColoredMap = false;
        }

        if (m_useColoredMap) {
#ifdef COLOR_OCTOMAP_SERVER
            RCLCPP_WARN(
                this->get_logger(),
                "Using RGB color registration (if information available)");
#else
            std::string msg = std::string("Colored map requested in launch file") +
                " - node not running/compiled to support colors, " +
                "please define COLOR_OCTOMAP_SERVER and recompile or launch " +
                "the octomap_color_server node";
            RCLCPP_WARN(this->get_logger(), msg.c_str());
#endif
        }

        // initialize octomap object & params
        m_octree = std::make_shared<OcTreeT>(m_res);
        m_octree->setProbHit(probHit);
        m_octree->setProbMiss(probMiss);
        m_octree->setClampingThresMin(thresMin);
        m_octree->setClampingThresMax(thresMax);
        m_treeDepth = m_octree->getTreeDepth();
        m_maxTreeDepth = m_treeDepth;
        m_gridmap.info.resolution = m_res;

        double r = this->declare_parameter("color/r", 0.0);
        double g = this->declare_parameter("color/g", 0.0);
        double b = this->declare_parameter("color/b", 1.0);
        double a = this->declare_parameter("color/a", 1.0);
        m_color.r = r;
        m_color.g = g;
        m_color.b = b;
        m_color.a = a;

        r = this->declare_parameter("color_free/r", 0.0);
        g = this->declare_parameter("color_free/g", 1.0);
        b = this->declare_parameter("color_free/b", 0.0);
        a = this->declare_parameter("color_free/a", 1.0);
        m_colorFree.r = r;
        m_colorFree.g = g;
        m_colorFree.b = b;
        m_colorFree.a = a;
        
        m_publishFreeSpace = this->declare_parameter(
            "publish_free_space", m_publishFreeSpace);

        // definition of the bool for binary and full octomap, centers pointcloud and 2d map
        publishFreeCells = this->declare_parameter(
            "publish_free_cells", publishFreeCells);
        publishOctomapBinary = this->declare_parameter(
            "publish_octomap_binary", publishOctomapBinary);
        publishOctomapFull = this->declare_parameter(
            "publish_octomap_full", publishOctomapFull);
        publishCentersPointcloud = this->declare_parameter(
            "publish_centers_pointcloud", publishCentersPointcloud);
        publish2DProjectedMap = this->declare_parameter(
            "publish_2d_projected_map", publish2DProjectedMap);

        messageFilterQueue = this->declare_parameter(
            "message_filter_queue", messageFilterQueue);

        partialPointcloudSubscription = this->declare_parameter(
            "partial_pointcloud_subscription", partialPointcloudSubscription); 

        std::string msg = std::string("Publishing non-latched (topics are only)") +
                    "prepared as needed, will only be re-published on map change";
        RCLCPP_INFO(this->get_logger(), msg.c_str());

        RCLCPP_INFO(this->get_logger(), "Frame Id %s", m_worldFrameId.c_str());
        RCLCPP_INFO(this->get_logger(), "Resolution %.2f", m_res);
        

        RCLCPP_INFO(this->get_logger(), "Octomap server's parameters and variables initialized.");



        RCLCPP_INFO(this->get_logger(), "Extended octomap server's constructor started.");



        // Initialization of the parameters
        processFreeSpace = this->declare_parameter(
            "process_free_space", processFreeSpace);
        if (processFreeSpace) {
            RCLCPP_INFO(this->get_logger(), "Computation of free voxels will not be executed.");

        }
        publishConfidence = this->declare_parameter(
            "publish_confidence", publishConfidence);
        publishSemantic = this->declare_parameter(
            "publish_semantic", publishSemantic);
        publishInstances = this->declare_parameter(
            "publish_instances", publishInstances);

        
        segmentedPointcloudSubscription = this->declare_parameter(
            "segmented_pointcloud_subscription", segmentedPointcloudSubscription);
        segmentedPointcloudsArraySubscription = this->declare_parameter(
            "segmented_pointclouds_array_subscription", segmentedPointcloudsArraySubscription);

        insertCloudActive = this->declare_parameter(
            "insert_cloud_init", insertCloudActive);
        insertSegmentedActive = this->declare_parameter(
            "insert_segmented_init", insertSegmentedActive);

        segmentedPointcloudOutlierRemoval = this->declare_parameter(
            "segmented_pointcloud_outlier_removal", segmentedPointcloudOutlierRemoval);

        useFrequencyThreshold = this->declare_parameter(
            "use_frequency_threshold", useFrequencyThreshold);
        
        frequencyThreshold = this->declare_parameter(
            "frequency_threshold", frequencyThreshold);

        outlier_detection = this->declare_parameter(
            "outlier_detection", outlier_detection);

        search_neighboorhood_ray = this->declare_parameter(
            "search_neighboorhood_ray", search_neighboorhood_ray);

        correction_neighboorhood_ray = this->declare_parameter(
            "correction_neighboorhood_ray", correction_neighboorhood_ray);

        outlier_threshold = this->declare_parameter(
            "outlier_threshold", outlier_threshold);

        weighted_confidence = this->declare_parameter(
            "weighted_confidence", weighted_confidence);


        
        // Case when the semantic segmentation is required
        if (segmentedPointcloudsArraySubscription or segmentedPointcloudSubscription){

            // Initialization of the map for the additional semantic information
            extended_octomap_map = std::make_shared<ExtendedOctomapMap>();
        }

        
        
        if (segmentedPointcloudsArraySubscription){
            // Initialization of the map to store the key of the voxel where there are collision between points of different instances
            collisionKeys = std::make_shared<CollisionOcTreeKeys>();
            // Initialization of the variable to keep track of the instances
            currentMaxInstance = 1;   
        }  

        RCLCPP_INFO(this->get_logger(), "Extended octomap server's parameters and variables initialized.");


    }


    /**
     * @brief Destructor for the ExtendedOctomapServer class.
     * 
     * This destructor cleans up the ExtendedOctomapServer class, releasing resources and memory allocated during
     * the lifetime of the class. It also performs cleanup operations such as clearing the OctoMap object and
     * releasing the tf2 buffer and listener.
     */
    ExtendedOctomapServer::~ExtendedOctomapServer(){}



    /**
     * @brief Creates the publishers and subscribers for the ExtendedOctomapServer class.
     * 
     * This function creates the publishers and subscribers for the ExtendedOctomapServer class, setting up the
     * necessary communication channels for handling OctoMap data and extended information. It creates publishers
     * for binary and full OctoMaps, point clouds, and 2D maps, as well as subscribers for point clouds and segmented
     * point clouds. It also creates services for handling OctoMap data requests.
     */
    void ExtendedOctomapServer::createPubSub() {

        // Publishers

        RCLCPP_INFO(this->get_logger(), "Creating octomap server publishers...");


        rclcpp::QoS qos(rclcpp::KeepLast(3));
        if (publishOctomapBinary){
            this->m_binaryMapPub = this->create_publisher<
                octomap_msgs::msg::Octomap>("octomap_binary", qos);
        }
        if (publishOctomapFull){
            this->m_fullMapPub = this->create_publisher<
                octomap_msgs::msg::Octomap>("octomap_full", qos);
        }
        if (publishCentersPointcloud){
            this->m_pointCloudPub = this->create_publisher<
                sensor_msgs::msg::PointCloud2>(
                    "octomap_point_cloud_centers", qos);
        }
        if (publish2DProjectedMap){
            this->m_mapPub = this->create_publisher<
                nav_msgs::msg::OccupancyGrid>("projected_map", qos);
        }
        
        
        RCLCPP_INFO(this->get_logger(), "Octomap server publishers created.");

        RCLCPP_INFO(this->get_logger(), "Creating extended octomap server publishers...");


        if (publishConfidence){
            this->confidenceMarkerPub = this->create_publisher<
                visualization_msgs::msg::MarkerArray>(
                    "confidence_cells_vis", qos);
            RCLCPP_INFO(this->get_logger(), "Publisher of confidence markers created.");
        }
        if (publishSemantic){
            this->semanticClassMarkerPub = this->create_publisher<
                visualization_msgs::msg::MarkerArray>(
                    "semantic_class_cells_vis", qos);
            RCLCPP_INFO(this->get_logger(), "Publisher of semantic classes markers created.");
        }
        if (publishInstances){
            this->instancesMarkerPub = this->create_publisher<
                visualization_msgs::msg::MarkerArray>(
                    "instances_cells_vis", qos);
            RCLCPP_INFO(this->get_logger(), "Publisher of instances markers created.");
        }


        RCLCPP_INFO(this->get_logger(), "Extended octomap server publishers created.");



        // Subscribers


        RCLCPP_INFO(this->get_logger(), "Creating octomap server subscribers...");


        this->m_pointCloudSub = std::make_shared<
            message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
                this, "cloud_in", rmw_qos_profile_sensor_data);
        this->fullPointcloudSub = std::make_shared<
            message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
                this, "cloud_in", rmw_qos_profile_sensor_data);

        // If the partial pointcloud is not required, the subscriber to the partial tf is created using 
        // the tf2_ros::MessageFilter
        if (!partialPointcloudSubscription){
            auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
                this->get_node_base_interface(),
                this->get_node_timers_interface());
            this->buffer_->setCreateTimerInterface(create_timer_interface);
            
            this->m_tfPointCloudSub = std::make_shared<tf2_ros::MessageFilter<
                sensor_msgs::msg::PointCloud2>>(
                    *buffer_, m_worldFrameId, messageFilterQueue,
                    this->get_node_logging_interface(),
                    this->get_node_clock_interface(),
                    std::chrono::seconds(1));
            this->m_tfPointCloudSub->connectInput(*m_pointCloudSub);
            this->m_tfPointCloudSub->registerCallback(
                std::bind(&ExtendedOctomapServer::insertCloudCallback, this, ph::_1));
            RCLCPP_INFO(this->get_logger(), "Subscription to pointcloud and tf topic done.");


        }
        // If the partial pointcloud is required, the subscriber to the partial tf is created using a synchronizer
        else{  
            this->partialTfSub = std::make_shared<
                message_filters::Subscriber<geometry_msgs::msg::TransformStamped>>(
                    this, "partial_tf", rmw_qos_profile_sensor_data);
            
            // This synchronizer synchronizes the pointcloud coming from /cloud_in (that is not a complete but a partial pointcloud) and the segmented tf
            sync_partial_pointcloud_ = std::make_shared<SynchronizerPartial>(SyncPolicyPartial(5), *m_pointCloudSub, *partialTfSub); 
            sync_partial_pointcloud_->registerCallback(
                std::bind(
                    &ExtendedOctomapServer::insertPartialCloudCallback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));
                        
            RCLCPP_INFO(this->get_logger(), "Subscription to partial pointcloud and partial tf topic done.");
        }
    

        
        if (publishOctomapBinary){
            this->m_octomapBinaryService = this->create_service<OctomapSrv>(
                "octomap_binary",
                std::bind(&ExtendedOctomapServer::octomapBinarySrv, this, ph::_1, ph::_2));
        }
        if (publishOctomapFull){
            this->m_octomapFullService = this->create_service<OctomapSrv>(
                "octomap_full",
                std::bind(&ExtendedOctomapServer::octomapFullSrv, this, ph::_1, ph::_2));
        }



        RCLCPP_INFO(this->get_logger(), "Creating extended octomap server subscribers...");

        // If the segmented pointclouds array is required, the subscriber to the segmented pointclouds array and 
        // segmented tf is created using a synchronizer
        if (segmentedPointcloudsArraySubscription){

            this->segmentedPointcloudsArraySub = std::make_shared<
                message_filters::Subscriber<av_interfaces::msg::PointcloudArray>>(
                    this, "segmented_pointclouds_array", rmw_qos_profile_sensor_data);

            this->segmentedPointcloudsArrayTfSub = std::make_shared<
                message_filters::Subscriber<geometry_msgs::msg::TransformStamped>>(
                    this, "segmented_pointclouds_array_tf", rmw_qos_profile_sensor_data);

            sync_segmented_pointclouds_array_ = std::make_shared<SynchronizerArray>(SyncPolicyArray(5), *segmentedPointcloudsArraySub, *segmentedPointcloudsArrayTfSub, *fullPointcloudSub);
            sync_segmented_pointclouds_array_->registerCallback(
                std::bind(
                    &ExtendedOctomapServer::insertSegmentedPointcloudsArrayCallback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3
                    ));
                        
            RCLCPP_INFO(this->get_logger(), "Subscription to segmented pointclouds array and segmented tf topic done.");
        }


        
        if (segmentedPointcloudSubscription){
            
            this->segmentedPointcloudSub = std::make_shared<
                message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
                    this, "segmented_pointcloud", rmw_qos_profile_sensor_data);
            
            this->segmentedPointcloudTfSub = std::make_shared<
                message_filters::Subscriber<geometry_msgs::msg::TransformStamped>>(
                    this, "segmented_pointcloud_tf", rmw_qos_profile_sensor_data);
            
            sync_segmented_pointcloud_ = std::make_shared<Synchronizer>(SyncPolicy(5), *segmentedPointcloudSub, *segmentedPointcloudTfSub, *fullPointcloudSub);
            sync_segmented_pointcloud_->registerCallback(
                std::bind(
                    &ExtendedOctomapServer::insertSegmentedPointcloudCallback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3
                    ));
                        
            RCLCPP_INFO(this->get_logger(), "Subscription to segmented pointcloud and segmented tf topic done.");
        }



        


        // Services

        this->m_clearBBXService = this->create_service<BBXSrv>(
            "clear_bbx",
            std::bind(&ExtendedOctomapServer::clearBBXSrv, this, ph::_1, ph::_2));
        this->m_resetService = this->create_service<std_srvs::srv::Empty>(
            "reset", std::bind(&ExtendedOctomapServer::resetSrv, this, ph::_1, ph::_2));

        // Case when the semantic segmentation is required
        if (segmentedPointcloudsArraySubscription or segmentedPointcloudSubscription){
            // Initialize the service for the activation of the segmented callbacks
            insertSegmentedActiveService_ = this->create_service<std_srvs::srv::SetBool>(
                "set_insert_segmented_active",
                std::bind(&ExtendedOctomapServer::setinsertSegmentedActive, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        }
    
        // Initialize the service for the activation of the cloud callback
        insertCloudActiveService_ = this->create_service<std_srvs::srv::SetBool>(
            "set_insert_cloud_active",
            std::bind(&ExtendedOctomapServer::setInsertCloudActive, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        saveOctomapDataService_ = this->create_service<std_srvs::srv::SetBool>(
            "save_octomap_data_service",
            std::bind(&ExtendedOctomapServer::saveOctomapDataServiceCallback, this, std::placeholders::_1, std::placeholders::_2));


        RCLCPP_INFO(this->get_logger(), "Services initialized.");

        RCLCPP_INFO(this->get_logger(), "The initialization is completed.");


    }



    /**
     * @brief Creates the visualization publishers for the ExtendedOctomapServer class.
     * 
     * This function creates the visualization publishers for the ExtendedOctomapServer class, setting up the
     * necessary communication channels for visualizing OctoMap data and extended information. It creates publishers
     * for occupied and free cells, confidence markers, semantic class markers, and instance markers.
     */
    void ExtendedOctomapServer::createVisualizations(){

        // Visualization publishers

        RCLCPP_INFO(this->get_logger(), "Creating extended octomap server visualization publishers...");


        rclcpp::QoS qos(rclcpp::KeepLast(3));
        this->m_markerPub = this->create_publisher<
            visualization_msgs::msg::MarkerArray>(
                "occupied_cells_vis_array", qos);
        RCLCPP_INFO(this->get_logger(), "Publisher of occupied markers created.");

        if (publishFreeCells){
            this->m_fmarkerPub = this->create_publisher<
                visualization_msgs::msg::MarkerArray>(
                    "free_cells_vis_array", qos);
            RCLCPP_INFO(this->get_logger(), "Publisher of unoccupied markers created.");
        }

        if (publishConfidence){
            this->confidenceMarkerPub = this->create_publisher<
                visualization_msgs::msg::MarkerArray>(
                    "confidence_cells_vis", qos);
            RCLCPP_INFO(this->get_logger(), "Publisher of confidence markers created.");
        }
        if (publishSemantic){
            this->semanticClassMarkerPub = this->create_publisher<
                visualization_msgs::msg::MarkerArray>(
                    "semantic_class_cells_vis", qos);
            RCLCPP_INFO(this->get_logger(), "Publisher of semantic classes markers created.");
        }
        if (publishInstances){
            this->instancesMarkerPub = this->create_publisher<
                visualization_msgs::msg::MarkerArray>(
                    "instances_cells_vis", qos);
            RCLCPP_INFO(this->get_logger(), "Publisher of instances markers created.");
        }
        

        RCLCPP_INFO(this->get_logger(), "The initialization of the visualization tools is completed.");


    }



    /**
     * @brief Callback function for the insertion of a pointcloud used to update the occupancy OctoMap.
     * 
     * The function takes a sensor_msgs/PointCloud2
     * point cloud message as input and inserts it into a PCL point cloud object. Coordinate
     * frame transformations are performed, specifically between the sensor frame and the world
     * frame. Filters are defined to remove points outside a specified range along each dimension
     * (x, y, z) of the point cloud. If ground filtering is enabled, the point cloud is filtered to
     * remove ground points, creating separate ground and non-ground point clouds.
     * 
     * @param cloud The pointcloud message to be processed.
     */
    void ExtendedOctomapServer::insertCloudCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud){

        // Variable to handle a service called to activate and deactivate the insertion of the pointcloud
        if (!insertCloudActive){
            return;
        }
        

        auto start = std::chrono::steady_clock::now();
        
        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertCloudCallback] Pointcloud callback started.");


        // Ground filtering in base frame
        PCLPointCloud pc; // input cloud for filtering and ground-detection
        pcl::fromROSMsg(*cloud, pc);
        
        Eigen::Matrix4f sensorToWorld; // matrix of size 4 composed of float
        geometry_msgs::msg::TransformStamped sensorToWorldTf;
        try {
            RCLCPP_DEBUG(this->get_logger(), "Can transform?");
            if (!this->buffer_->canTransform(
                    m_worldFrameId, cloud->header.frame_id,
                    cloud->header.stamp)) {
                RCLCPP_ERROR(this->get_logger(), "Can't transform");
                throw "Failed";
            }
            
            RCLCPP_DEBUG(this->get_logger(), "Can transform");

            sensorToWorldTf = this->buffer_->lookupTransform(
                m_worldFrameId, cloud->header.frame_id,
                cloud->header.stamp);
            sensorToWorld = transformAsMatrix(sensorToWorldTf);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s",ex.what());
            return;
        }

        // Set up filter for height range, also removes NANs:
        pcl::PassThrough<PCLPoint> pass_x;
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
        pcl::PassThrough<PCLPoint> pass_y;
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
        pcl::PassThrough<PCLPoint> pass_z;
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

        PCLPointCloud pc_ground; // segmented ground plane
        PCLPointCloud pc_nonground; // everything else
        
        if (m_filterGroundPlane) {
            geometry_msgs::msg::TransformStamped baseToWorldTf;
            geometry_msgs::msg::TransformStamped sensorToBaseTf;
            
            try {
                if (!this->buffer_->canTransform(
                    m_baseFrameId, cloud->header.frame_id,
                    cloud->header.stamp)) {
                    throw "Failed";
                }

                sensorToBaseTf = this->buffer_->lookupTransform(
                    m_baseFrameId, cloud->header.frame_id,
                    cloud->header.stamp);
                baseToWorldTf = this->buffer_->lookupTransform(
                    m_worldFrameId, m_baseFrameId, cloud->header.stamp);
            } catch (tf2::TransformException& ex) {
                std::string msg = std::string("Transform error for ground plane filter") +
                    "You need to set the base_frame_id or disable filter_ground.";
                RCLCPP_ERROR(this->get_logger(), "%s %s", msg.c_str(), ex.what());
                return;
            }

            Eigen::Matrix4f sensorToBase =
                transformAsMatrix(sensorToBaseTf);
            Eigen::Matrix4f baseToWorld = 
                transformAsMatrix(baseToWorldTf);

            // transform pointcloud from sensor frame to fixed robot frame
            pcl::transformPointCloud(pc, pc, sensorToBase);
            pass_x.setInputCloud(pc.makeShared());
            pass_x.filter(pc);
            pass_y.setInputCloud(pc.makeShared());
            pass_y.filter(pc);
            pass_z.setInputCloud(pc.makeShared());
            pass_z.filter(pc);
            filterGroundPlane(pc, pc_ground, pc_nonground);

            // transform clouds to world frame for insertion
            pcl::transformPointCloud(pc_ground, pc_ground, baseToWorld);
            pcl::transformPointCloud(pc_nonground, pc_nonground, baseToWorld);
        } else {
            // directly transform to map frame:
            pcl::transformPointCloud(pc, pc, sensorToWorld);
            
            // just filter height range:
            pass_x.setInputCloud(pc.makeShared());
            pass_x.filter(pc);
            pass_y.setInputCloud(pc.makeShared());
            pass_y.filter(pc);
            pass_z.setInputCloud(pc.makeShared());
            pass_z.filter(pc);

            pc_nonground = pc;
            // pc_nonground is empty without ground segmentation
            pc_ground.header = pc.header;
            pc_nonground.header = pc.header;
        }
        
        insertScan(sensorToWorldTf.transform.translation,
                   pc_ground, pc_nonground);

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        RCLCPP_INFO(this->get_logger(), "Time lapse [from receiving pointcloud sensor message to final octomap insertion] %f", elapsed_seconds.count());
        
        publishAll(cloud->header.stamp);



        if (publishConfidence){        
            publishConfidenceMarkers(cloud->header.stamp);
        }

        if (publishSemantic){
            publishSemanticClassMarkers(cloud->header.stamp);
        }

        if (publishInstances){
            publishInstancesMarkers(cloud->header.stamp);
        }
        
    }


    /**
     * @brief Callback function for the insertion of a segmented pointcloud used to update the occupancy OctoMap.
     * 
     * This function is introduced to calculate the occupancy OctoMap starting from a smaller point cloud in order to 
     * decrease the computational load. The resulting occupancy OctoMap is composed of the portion of space related to 
     * the segmented image coming from the segmentation process. This procedure can be useful for testing
     * purposes whenever it is not necessary to have an occupancy OctoMap containing the entire occupied portion 
     * of the space. If the color-filtering approach is used, the point cloud
     * produced is simply passed to this function. If the zero-shot segmentation approach is used,
     * instead of passing the array of point clouds, the point cloud produced from the merged
     * masks needs to be passed.
     * 
     * @param cloud The full pointcloud message to be processed.
     * @param segmented_tf the tf related to the pointcloud message to be processed.
     */
    void ExtendedOctomapServer::insertPartialCloudCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,
        const geometry_msgs::msg::TransformStamped::ConstSharedPtr &segmented_tf
        ){

        if (!insertCloudActive){
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertPartialPointcloudCallback] Partial pointcloud callback started.");

        //
        // ground filtering in base frame
        //
        auto start = std::chrono::steady_clock::now();
        PCLPointCloud pc; // input cloud for filtering and ground-detection
        pcl::fromROSMsg(*cloud, pc);
        
        Eigen::Matrix4f sensorToWorld; // matrix of size 4 composed of float
        geometry_msgs::msg::TransformStamped sensorToWorldTf = *segmented_tf;
        try {
            sensorToWorld = transformAsMatrix(sensorToWorldTf);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s",ex.what());
            return;
        }

        // set up filter for height range, also removes NANs:
        pcl::PassThrough<PCLPoint> pass_x;
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
        pcl::PassThrough<PCLPoint> pass_y;
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
        pcl::PassThrough<PCLPoint> pass_z;
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

        PCLPointCloud pc_ground; // segmented ground plane
        PCLPointCloud pc_nonground; // everything else
        
        
        // directly transform to map frame (TODO: redo the same procedures of the octomap server when filter ground is true)
        pcl::transformPointCloud(pc, pc, sensorToWorld);
        
        // just filter height range:
        pass_x.setInputCloud(pc.makeShared());
        pass_x.filter(pc);
        pass_y.setInputCloud(pc.makeShared());
        pass_y.filter(pc);
        pass_z.setInputCloud(pc.makeShared());
        pass_z.filter(pc);


        // Pointcloud outlier removal
        if (segmentedPointcloudOutlierRemoval){
            // Create the filtering object
            pcl::StatisticalOutlierRemoval<PCLPoint> sor;

            // Directly create a shared_ptr from segmented_pc
            std::shared_ptr<PCLPointCloud> input_cloud(new PCLPointCloud(pc));

            sor.setInputCloud(input_cloud);
            sor.setMeanK(50);
            sor.setStddevMulThresh(1.0);
            sor.filter(pc);
        }


        pc_nonground = pc;
        // pc_nonground is empty without ground segmentation
        pc_ground.header = pc.header;
        pc_nonground.header = pc.header;
        
        
        ExtendedOctomapServer::insertScan(sensorToWorldTf.transform.translation,
                   pc_ground, pc_nonground);

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        RCLCPP_INFO(this->get_logger(), "Time lapse [from receiving pointcloud sensor message to final octomap insertion] %f", elapsed_seconds.count());
        
        publishAll(cloud->header.stamp);

        if (publishConfidence){        
            publishConfidenceMarkers(cloud->header.stamp);
        }

        if (publishSemantic){
            publishSemanticClassMarkers(cloud->header.stamp);
        }

        if (publishInstances){
            publishInstancesMarkers(cloud->header.stamp);
        }
        
    }


    /**
     * @brief Callback function for the insertion of a pointcloud used to update the occupancy OctoMap.
     * 
     * First, the function obtains the ground and non-ground point cloud objects. Then, two unordered
     * sets are created, containing the addresses or keys of unoccupied and occupied voxels.
     * A loop iterates through the ground point cloud object. For each point, a ray is generated
     * from the sensor origin to itself. The ray is inserted into the unoccupied voxel keys’ set.
     * A loop iterates through the non-ground point cloud object. For each point, a ray from
     * the sensor origin to itself is created. This ray is inserted into the unoccupied voxel keys’
     * set, while the point is added to the occupied voxel keys’ set. The discrete voxel keys
     * representing the position of the minimum and maximum values of the bounding box of
     * all known space in the x, y, and z dimensions are updated.
     * Next, a loop iterates through the unoccupied voxel keys’ set. Each key is checked against
     * the occupied voxel keys’. If a key is only in the unoccupied voxel keys’ set, the occupancy
     * probability is updated and will presumably decrease toward 0. A loop iterates through
     * the occupied voxel keys’ set.
     * 
     * If segmentation is required, the keys of the hash map related to the semantic OctoMap need to be initialized.
     * A loop through the occupied voxels’ keys calculated in this specific scan is performed. A
     * check regarding the presence of each key in the global set containing the occupied voxels’
     * keys found during all computation (even during the previous scans) is executed. If it is
     * not present, this means that the key has been found in the current scan and needs to be
     * added to the global set. In this way, a set stores the occupied voxels’ keys found during
     * the entire execution of the plugin.
     * Next, a loop through the global occupied voxels’ keys is executed. If a key is not present
     * in the hash map related to the semantic OctoMap, a new entry is added with this key,
     * setting the value in this way: the semantic class is ’none’, the confidence is 0.0, the
     * instance number is 0, the colors related to class, confidence and instance are white.
     * 
     * @param sensorOriginTf the tf related to the pointcloud message to be processed.
     * @param ground the ground pointcloud message to be processed.
     * @param nonground the non-ground pointcloud message to be processed.
     */
    void ExtendedOctomapServer::insertScan(
        const geometry_msgs::msg::Vector3 &sensorOriginTf,
        const PCLPointCloud& ground,
        const PCLPointCloud& nonground) 
    {
        octomap::point3d sensorOrigin = pointTfToOctomap(sensorOriginTf);
        
        if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)
            || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax)) {

            RCLCPP_WARN(this->get_logger(),
                        "Could not generate Key for origin");
        }

#ifdef COLOR_OCTOMAP_SERVER
        unsigned char* colors = new unsigned char[3];
#endif
        
        // instead of direct scan insertion, compute update to filter ground:
        octomap::KeySet free_cells, occupied_cells;

        // insert ground points only as free:
        for (auto it = ground.begin(); it != ground.end(); ++it) {
            octomap::point3d point(it->x, it->y, it->z);
            // maxrange check
            if ((m_maxRange > 0.0) && ((point - sensorOrigin).norm() > m_maxRange) ) {
                point = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
            }

            // only clear space (ground points)
            if (processFreeSpace){
                if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)) {
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());
                }
            }

            octomap::OcTreeKey endKey;
            if (m_octree->coordToKeyChecked(point, endKey)) {
                updateMinKey(endKey, m_updateBBXMin);
                updateMaxKey(endKey, m_updateBBXMax);
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "Could not generate Key for endpoint");
            }
        }

        auto start = std::chrono::steady_clock::now();
        
        // all other points: free on ray, occupied on endpoint:
        for (auto it = nonground.begin(); it != nonground.end(); ++it) {
            octomap::point3d point(it->x, it->y, it->z);
            // maxrange check            
            if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange)) {
                // free cells
                if (processFreeSpace){
                    if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)) {
                        free_cells.insert(m_keyRay.begin(), m_keyRay.end());
                    }
                }

                // occupied endpoint
                octomap::OcTreeKey key;
                if (m_octree->coordToKeyChecked(point, key)) {
                    occupied_cells.insert(key);

                    updateMinKey(key, m_updateBBXMin);
                    updateMaxKey(key, m_updateBBXMax);

#ifdef COLOR_OCTOMAP_SERVER
                // NB: Only read and interpret color if it's an occupied node
                    m_octree->averageNodeColor(it->x, it->y, it->z,
                                               it->r, it->g, it->b);
#endif
                }
            } else {
                if (processFreeSpace){
                    // ray longer than maxrange:;
                    octomap::point3d new_end = sensorOrigin +
                        (point - sensorOrigin).normalized() * m_maxRange;
                    if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)) {
                        free_cells.insert(m_keyRay.begin(), m_keyRay.end());
                        octomap::OcTreeKey endKey;
                        if (m_octree->coordToKeyChecked(new_end, endKey)) {
                            free_cells.insert(endKey);
                            updateMinKey(endKey, m_updateBBXMin);
                            updateMaxKey(endKey, m_updateBBXMax);
                        } else {
                            RCLCPP_ERROR(this->get_logger(),
                                        "Could not generate Key for endpoint");
                        }
                    }
                }
            }
        }


        
        if (processFreeSpace){
            // mark free cells only if not seen occupied in this cloud
            for(auto it = free_cells.begin(), end=free_cells.end();
                it!= end; ++it){
                if (occupied_cells.find(*it) == occupied_cells.end()){ // if the free cell is not found in occupied cell, and so if this cell is a real free cell
                    m_octree->updateNode(*it, false);
                }
            }
        }

        // now mark all occupied cells:
        for (auto it = occupied_cells.begin(),
                 end=occupied_cells.end(); it!= end; it++) {
            m_octree->updateNode(*it, true);
        }

        octomap::point3d minPt, maxPt;
        minPt = m_octree->keyToCoord(m_updateBBXMin);
        maxPt = m_octree->keyToCoord(m_updateBBXMax);
        
        if (m_compressMap) {
            m_octree->prune();
        }



        // Case when the segmentation is required
        if (segmentedPointcloudsArraySubscription or segmentedPointcloudSubscription){

            // populate the global sets. No need to check if it already exists
            global_occupied_cells.insert(occupied_cells.begin(), occupied_cells.end());
            if (processFreeSpace){
                global_free_cells.insert(free_cells.begin(), free_cells.end());
            }

            // Insert into the ExtendedOctomapMap keys all the occupied OctKeys
            // and set the class as none if the octkey does not exist yet
            for(const auto& key : global_occupied_cells) {
                if (extended_octomap_map->find(key) == extended_octomap_map->end()) {
                (*extended_octomap_map)[key] = ExtendedOctomapData(colorMap);  
                }                    
            }
        }

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        if (processFreeSpace){
            RCLCPP_INFO(this->get_logger(), "[INSERT-SCAN] Duration of the insertion of the PCL pointcloud in the free and occupied octree nodes: %f", elapsed_seconds.count());
        }
        else{
            RCLCPP_INFO(this->get_logger(), "[INSERT-SCAN] Duration of the insertion of the PCL pointcloud in the occupied octree nodes: %f", elapsed_seconds.count());
        }
        
#ifdef COLOR_OCTOMAP_SERVER
        if (colors) {
            delete[] colors;
            colors = NULL;
        }
#endif
    }



    /**
     * @brief Callback used to insert a single segmented-image point cloud into the semantic OctoMap.
     * 
     * The function takes as input a sensor_msgs/PointCloud2 segmented-image point cloud message related 
     * to the color-filtering segmentation. Coordinate frame transformations are performed, specifically 
     * between the sensor frame and the world frame. Filters are defined to remove points outside a 
     * specified range along each dimension (x, y, z) of the point cloud. A loop through this segmented 
     * point cloud is performed. Each point is converted into a voxel, and if the voxel’s key is present in 
     * the hash map related to the semantic OctoMap, it is skipped, otherwise the semantic class of value of 
     * this key is set with a string different from ’none’. In this way, all the points that have been segmented
     * using the color-filtering approach correspond to semantic voxels.
     * 
     * @param segmented_pointcloud The segmented pointcloud message to be processed.
     * @param segmented_tf the tf related to the pointcloud message to be processed.
     * @param cloud the full pointcloud message to be processed.
     */
    void ExtendedOctomapServer::insertSegmentedPointcloudCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &segmented_pointcloud, 
        const geometry_msgs::msg::TransformStamped::ConstSharedPtr &segmented_tf,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud
    ){

        // If the paramter to activate the callback is false, the callback will be skipped
        if (!insertSegmentedActive){
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudCallback] Segmented pointcloud callback started.");

        // From pointcloud message to Pointcloud data structure
        PCLPointCloud segmented_pc;
        pcl::fromROSMsg(*segmented_pointcloud, segmented_pc);

        // From pointcloud message to Pointcloud data structure
        PCLPointCloud pc;
        pcl::fromROSMsg(*cloud, pc);

        // Conversion of the pointcloud from sensor frame to world frame
        Eigen::Matrix4f sensorToWorld;
        geometry_msgs::msg::TransformStamped sensorToWorldTf = *segmented_tf;
        try {
            sensorToWorld = transformAsMatrix(sensorToWorldTf);
        } catch (tf2::TransformException &ex) {
            return;
        } catch (const std::exception& e) {
            // This will catch standard exceptions
            RCLCPP_ERROR(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudCallback] %s",e.what());
        } catch (...) {
            // This will catch all other exceptions
            RCLCPP_ERROR(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudCallback] Generic error.");
        }

        pcl::transformPointCloud(segmented_pc, segmented_pc, sensorToWorld);


        // Set up filtering of the pointcloud based on the parameters related to the min and max possible values of the points
        pcl::PassThrough<PCLPoint> pass_x;
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
        pcl::PassThrough<PCLPoint> pass_y;
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
        pcl::PassThrough<PCLPoint> pass_z;
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

        pass_x.setInputCloud(segmented_pc.makeShared());
        pass_x.filter(segmented_pc);
        pass_y.setInputCloud(segmented_pc.makeShared());
        pass_y.filter(segmented_pc);
        pass_z.setInputCloud(segmented_pc.makeShared());
        pass_z.filter(segmented_pc);
        

        // Check if after filtering the pointcloud are empty. If yes, skip the iteration
        if (segmented_pc.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudCallback] Pointcloud after filtering is empty, skipping to next iteration.");
            return;
        }


        for (auto it = segmented_pc.begin(); it != segmented_pc.end(); ++it) {
            octomap::point3d point(it->x, it->y, it->z);                
            octomap::OcTreeKey key;

            if (m_octree->coordToKeyChecked(point, key)) {
                if (extended_octomap_map->find(key) == extended_octomap_map->end()) {
                    // if the key does not exists yet in the extended octomap map, initialize at zero
                   (*extended_octomap_map)[key] = ExtendedOctomapData(colorMap); 
                }
                (*extended_octomap_map)[key].setSemanticClass("mask", colorMap);
            }
        }

        if (publishConfidence){        
            publishConfidenceMarkers(segmented_pointcloud->header.stamp);
        }

        if (publishSemantic){
            publishSemanticClassMarkers(segmented_pointcloud->header.stamp);
        }

        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudCallback] Extended octomap map updated with segmented pointcloud.");
    }



    /**
     * @brief Callback related to the insertion of an array of point clouds obtained by the zero-shot segmentation process.
     * 
     * The function starts by taking as input the custom message containing the array of point clouds, the semantic class, 
     * and the dictionary of confidences. Each point cloud refers to a mask found during the zero-shot segmentation. 
     * For each point cloud, the main procedure is executed as follows:
     * 
     * 1. **Store Confidences and Semantic Class**: The confidences and the semantic class coming from the input custom message are stored.
     * 2. **Convert and Transform Point Cloud**: Each point cloud message sensor_msgs/PointCloud2 is converted into a PCL point cloud object, 
     *    transformed from sensor frame to world frame, and filtered based on the parameters related to the minimum and maximum possible values 
     *    admitted for the points. The outlier removal is applied to the point cloud object.
     * 3. **Loop Through Points**: A loop through the points is executed. If the node related to a point is not present in the hash map related 
     *    to the semantic OctoMap, a new entry is added with this key, setting the value in this way: the semantic class is 'none', the confidence 
     *    is 0.0, the instance number is 0, the colors related to class, confidence, and instance are white. The key of the voxel located at the 
     *    point is retrieved and stored in a utility set.
     * 4. **Store Voxel Keys**: At the end of the loop, the utility set contains all the keys of the voxels located at the point cloud being analyzed. 
     *    The next operations use this set and not the point cloud since it is more efficient.
     * 5. **Find Most Frequent Instance Number**: The most frequent instance number of the voxels inside the utility set is saved using a specific function. 
     *    The function stores the frequency of each instance number by iterating through the keys of the utility set, retrieving the associated instance 
     *    number, and incrementing the count for that instance number. The function finds the instance number with the highest frequency.
     * 6. **Handle Frequency Threshold**: The function has an optional feature controlled by the useFrequencyThreshold argument. This is useful when an 
     *    instance has the majority of voxels with instance number 0 (no semantic), but the actual instance is different from the non-semantic one (0). 
     *    If the most frequent instance number is 0, this instance number is removed, making the second the new most frequent instance number. The function 
     *    then compares the frequency of this second most frequent instance number to the original frequency of the instance number 0. If the second most 
     *    frequent instance number has a frequency that is at least a certain percentage (frequencyThreshold argument) of the original instance 0’s frequency, 
     *    the function returns the second most frequent instance number. Otherwise, it still returns the instance 0 as the most frequent one.
     * 7. **Update OctoMap Data**: The function iterates through the voxel keys of the utility set and updates the OctoMap data. The semantic information 
     *    (class, confidence, instance number, number of points present in the voxel associated with the object instance number) is stored in specific variables.
     *    - If the voxel instance number is equal to the most frequent instance number found using the previous procedure, its confidence and semantic class 
     *      are updated using max fusion. The number of points present in the voxel associated with the object instance number is also updated. If the instance 
     *      number of the voxel is 0, the instance needs to be initialized with the current maximum (and free) instance number.
     *    - If the voxel’s instance number is not equal to the most frequent instance number, it may happen that the most frequent instance number is 0, and 
     *      this means that the current instance has not been initialized. The voxel goes to the collision map to decide if it needs to remain with the already 
     *      existing instance number or be considered with the most frequent instance number.
     * 8. **Check for Voxel Collisions**: The function checks for voxel collisions. The collision map is a map where the keys are voxels, and the values are tuples: 
     *    one element refers to the current values of the voxel, and the other refers to the values that the voxel will obtain if it would be considered as part of 
     *    the instance defined by the most frequent instance number. Each element of the tuple has a count representing the number of points in the voxel related 
     *    to the current voxel’s instance and the instance identified by the most frequent instance number. This voxel is set with the instance number of the tuple 
     *    element that has the greater count value. The collision map is cleared to free memory and prepare for the next iteration.
     * 9. **Outlier Detection and Removal**: The final steps of the iteration are the outlier detection and removal in the semantic OctoMap and the function to 
     *    weight the confidence of the voxels of an instance with the same value.
     * 
     * @param segmented_pointclouds_array The segmented pointclouds array message to be processed.
     * @param segmented_tf The tf related to the pointcloud message to be processed.
     * @param cloud The full pointcloud message to be processed.
     */
    void ExtendedOctomapServer::insertSegmentedPointcloudsArrayCallback(
        const av_interfaces::msg::PointcloudArray::ConstSharedPtr &segmented_pointclouds_array, 
        const geometry_msgs::msg::TransformStamped::ConstSharedPtr &segmented_tf,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud
    ){


        // If the paramter to activate the callback is false, the callback will be skipped
        if (!insertSegmentedActive){
            return;
        }

        // The callback starts the execution
        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudsArrayCallback] Segmented pointclouds array callback started.");
        

        // If the input data composed of segmented pointclouds and relative confidences is empty, terminate the callback
        if (segmented_pointclouds_array->confidences.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudsArrayCallback] Segmented pointclouds array callback finished with no effect.");    
            return;   
        }

        // From pointcloud message to Pointcloud data structure
        PCLPointCloud pc;
        pcl::fromROSMsg(*cloud, pc);


        // If it is not empty, save confidence vector
        std::vector<float> confidences;
        for (auto& kv : segmented_pointclouds_array->confidences) {
            confidences.push_back(std::stof(kv.value));
        }

        // Save input semantic class
        std::string semantic_class = segmented_pointclouds_array->semantic_class;


        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudsArrayCallback] Iterating through instances (pointclouds)...");


        // Loop thorugh all the poitclouds inside the array
        for (size_t i = 0; i < segmented_pointclouds_array->pointclouds.size(); ++i){

            // Take the pointcloud and the confidence
            auto& segmented_pointcloud = segmented_pointclouds_array->pointclouds[i];
            float confidence = confidences[i];

            // // For debug save old confidence and new confidence
            // float old_confidence;
            // float new_confidence;
            // bool debug_var_filled = false;


            // Data structure to save all the octreekeys that contain the current pointcloud,
            // so that the next operation will be done iterating through this structure
            octomap::KeySet pointcloudKeys;

            

            // From pointcloud message to Pointcloud data structure
            PCLPointCloud segmented_pc;
            pcl::fromROSMsg(segmented_pointcloud, segmented_pc);

            // Conversion of the pointcloud from sensor frame to world frame
            Eigen::Matrix4f sensorToWorld;
            geometry_msgs::msg::TransformStamped sensorToWorldTf = *segmented_tf;
            try {
                sensorToWorld = transformAsMatrix(sensorToWorldTf);
            } catch (tf2::TransformException &ex) {
                return;
            } catch (const std::exception& e) {
                // This will catch standard exceptions
                RCLCPP_ERROR(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudsArrayCallback] %s",e.what());
            } catch (...) {
                // This will catch all other exceptions
                RCLCPP_ERROR(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudsArrayCallback] Generic error.");
            }

            pcl::transformPointCloud(segmented_pc, segmented_pc, sensorToWorld);

            RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudsArrayCallback] Pointcloud before the filtering contains %zu points", segmented_pc.size());


            // Set up filtering of the pointcloud based on the parameters related to the min and max possible values admitted of the points
            pcl::PassThrough<PCLPoint> pass_x;
            pass_x.setFilterFieldName("x");
            pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
            pcl::PassThrough<PCLPoint> pass_y;
            pass_y.setFilterFieldName("y");
            pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
            pcl::PassThrough<PCLPoint> pass_z;
            pass_z.setFilterFieldName("z");
            pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

            pass_x.setInputCloud(segmented_pc.makeShared());
            pass_x.filter(segmented_pc);
            pass_y.setInputCloud(segmented_pc.makeShared());
            pass_y.filter(segmented_pc);
            pass_z.setInputCloud(segmented_pc.makeShared());
            pass_z.filter(segmented_pc);



            // Check if after filtering the pointcloud are empty. If yes, skip the iteration
            if (segmented_pc.empty()) {
                RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudsArrayCallback] Pointcloud after filtering is empty, skipping to next iteration.");
                continue; // Go to the next iteration of the loop
            }


            // Pointcloud outlier removal
            if (segmentedPointcloudOutlierRemoval){
                // Create the filtering object
                pcl::StatisticalOutlierRemoval<PCLPoint> sor;

                // Directly create a shared_ptr from segmented_pc
                std::shared_ptr<PCLPointCloud> input_cloud(new PCLPointCloud(segmented_pc));

                sor.setInputCloud(input_cloud);
                sor.setMeanK(50);
                sor.setStddevMulThresh(1.0);
                sor.filter(segmented_pc);
            }



            // Save the octreekeys into the data structure. If they dont exists yet in the extended octomap map, insert and initialize at zero
            // If one exists into the pointcloudKeys, skip, since more points can be in the same key
            for (auto it = segmented_pc.begin(); it != segmented_pc.end(); ++it) {
                octomap::point3d point(it->x, it->y, it->z);                
                octomap::OcTreeKey key;

                if (m_octree->coordToKeyChecked(point, key)) { // find the key of the point
                    if (extended_octomap_map->find(key) == extended_octomap_map->end()) {
                        // if the key does not exists yet in the extended octomap map, initialize at zero
                       (*extended_octomap_map)[key] = ExtendedOctomapData(colorMap); 
                    }
                    // Try to insert the key. If the key is already present, the insertion will fail, but it's fine since we don't want duplicates.
                    pointcloudKeys.insert(key);
                }
            }

            RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudsArrayCallback] pointcloudKeys filled with %zu keys belonging to the pointcloud with %zu points", pointcloudKeys.size(), segmented_pc.size());

            // Save the most frequent instance value of the previous keys
            int most_frequent_instance = findMostFrequentInstance(*extended_octomap_map, pointcloudKeys);

            RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudsArrayCallback] The most frequent instance of these keys is %d", most_frequent_instance);


            
            // Variable to check if an instance has been inserted
            bool current_max_instance_used = false;

            RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudsArrayCallback] Iterating through keys...");

            // Iterate through the octreekeys related to the current pointcloud
            for(const auto& pt_key : pointcloudKeys) {
                
                // Get info of the key
                int pt_key_instance = (*extended_octomap_map)[pt_key].getInstance();
                std::string pt_key_class = (*extended_octomap_map)[pt_key].getSemanticClass();
                int pt_key_points_count = (*extended_octomap_map)[pt_key].getPointsCount();
                float pt_key_confidence = (*extended_octomap_map)[pt_key].getConfidence();

                // Case when the key has the same value as the major instance value of the entire pointcloud
                if (pt_key_instance == most_frequent_instance){

                    // // Debug: print the old confidence (only for the first key in the set)
                    // if (!debug_var_filled){
                    //     old_confidence = (*extended_octomap_map)[pt_key].getConfidence();
                    // }

                    (*extended_octomap_map)[pt_key].setConfidenceMaxFusion(semantic_class, colorMap, confidence);
                    int pt_key_count = countPointsInVoxel(segmented_pc, pt_key, m_octree);
                    (*extended_octomap_map)[pt_key].setPointsCount(pt_key_count);

                    // if the instance of this key is zero means that need to be initializated with the current max (and free) instance value
                    if (pt_key_instance == 0){
                        (*extended_octomap_map)[pt_key].setInstance(currentMaxInstance);
                        current_max_instance_used = true; // flag to say that this instance value has been used
                    }

                    // // Debug: print the old confidence (only for the first key in the set)
                    // if (!debug_var_filled){
                    //     new_confidence = (*extended_octomap_map)[pt_key].getConfidence();
                    //     RCLCPP_ERROR(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudsArrayCallback] Instance %d: Old conf=%f, Current conf=%f, New conf=%f", (*extended_octomap_map)[pt_key].getInstance(), old_confidence, confidence, new_confidence);
                    //     debug_var_filled = true;
                    // }
                }

                

                // Case when the key has not the same value as the major instance value of the entire pointcloud 
                else if (pt_key_instance != most_frequent_instance){
                    
                    // If the major instance value is zero, it means that the instance has not been initialized 
                    // and the voxel has already an instance, so it goes to the collision map
                    if(most_frequent_instance == 0){
                        int pt_key_current_count = countPointsInVoxel(segmented_pc, pt_key, m_octree); 
                        std::list<std::tuple<int, std::string, int, float>> tuplesList;
                        tuplesList.push_back(std::make_tuple(pt_key_instance, pt_key_class, pt_key_points_count, pt_key_confidence));
                        tuplesList.push_back(std::make_tuple(currentMaxInstance, semantic_class, pt_key_current_count, confidence));
                        (*collisionKeys)[pt_key] = tuplesList;
                    }
                    
                    // If there is already an instance but the current key has instance zero, means that the key is new
                    // and needs to be set with instance equal to the major one
                    // next pt can have this voxel also, and the collision will be handle well even for these keys
                    else if (most_frequent_instance != 0 && pt_key_instance == 0){
                        (*extended_octomap_map)[pt_key].setConfidenceMaxFusion(semantic_class, colorMap, confidence);
                        int pt_key_count = countPointsInVoxel(segmented_pc, pt_key, m_octree);
                        (*extended_octomap_map)[pt_key].setPointsCount(pt_key_count);
                        (*extended_octomap_map)[pt_key].setInstance(most_frequent_instance);
                    }
                }
            }


            // Increment the current max instance value if this variable has been used for at least one octreekey
            if (current_max_instance_used){
                currentMaxInstance++;
                RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudsArrayCallback] Instance incremented: %d", currentMaxInstance);
            }
            

            
        }

        // Check for the collisions octreekeys

        // At each iteration, the collisions of an octreekey are checked
        for (auto it = collisionKeys->begin(); it != collisionKeys->end(); ++it) {
            const auto& tuplesList = it->second;
            
            RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudsArrayCallback] Collision keys:");
            std::string logMsg = "Key: (" + std::to_string(it->first.k[0]) + ", " + std::to_string(it->first.k[1]) + ", " + std::to_string(it->first.k[2]) + ") Values: ";
            for (const auto& tuple : tuplesList) {
                logMsg += "Tuple: (";
                logMsg += std::to_string(std::get<0>(tuple)) + ", "; // int
                logMsg += std::get<1>(tuple) + ", "; // std::string
                logMsg += std::to_string(std::get<2>(tuple)) + ", "; // int
                logMsg += std::to_string(std::get<3>(tuple)); // float
                logMsg += ") ";
            }
            RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudsArrayCallback] %s", logMsg.c_str());

            
            // maxTupleIt will point to the tuple with the greatest third element, that is the points count value
            auto maxTupleIt = std::max_element(tuplesList.begin(), tuplesList.end(), 
                [](const std::tuple<int, std::string, int, float>& a, const std::tuple<int, std::string, int, float>& b) {
                    return std::get<2>(a) < std::get<2>(b); // Compare the third element that correspond to the points count values
                });
            
            // Set the values of the current octreekey
            (*extended_octomap_map)[it->first].setInstance(std::get<0>(*maxTupleIt));
            (*extended_octomap_map)[it->first].setConfidenceMaxFusion(std::get<1>(*maxTupleIt), colorMap, std::get<3>(*maxTupleIt));
            (*extended_octomap_map)[it->first].setPointsCount(std::get<2>(*maxTupleIt));
        }

        // Free the collision map
        collisionKeys->clear();




        // Outlier detection and removal
        // The algorithm starts by initializing a threshold from the outlier_threshold argument. 
        // This is used to determine whether a voxel should be considered an outlier.
        // The loop starts iterating through all the nodes of the occupied OctoMap.
        // • A search neighborhood is constructed around the current voxel. This neighborhood
        // is a cubic region centered on the voxel, extending search_neighborhood_ray units
        // in each direction. The algorithm iterates through each neighboring position and
        // checks if the corresponding voxel exists. If it does, the voxel’s key is added to the
        // search neighborhood set.
        // • Next, a count variable containing how many voxels in the search neighborhood have
        // the same instance number as the current voxel is stored. The threshold value is cal-
        // culated as a percentage of the total number of voxels in the search neighborhood. If
        // the count of neighbor voxels with the same instance number is below this threshold,
        // the voxel is considered an outlier.
        // • If the voxel is identified as an outlier, a correction neighborhood is
        // built similarly to the search neighborhood but with a different radius
        // correction_neighborhood_ray. The most frequent instance number in this cor-
        // rection neighborhood is calculated similarly to how it was done in Algorithm 3.1.
        // If the most frequent instance number in the correction neighborhood differs from
        // the current voxel’s instance number, the voxel is considered an outlier. The voxel’s
        // instance number is updated to the most frequent instance number from the correc-
        // tion neighborhood. Additionally, the voxel’s confidence, color, and semantic class
        // are updated based on the most frequent instance in the correction neighborhood.
        // This ensures that the voxel’s properties are consistent with its corrected instance.
        if (outlier_detection) {
            double threshold = outlier_threshold;
            // Iterate through all leaf nodes
            for (auto it = m_octree->begin(m_treeDepth), end = m_octree->end(); it != end; ++it) {
                octomap::OcTreeKey key = it.getKey();
                octomap::point3d current_key = m_octree->keyToCoord(key);
                Eigen::Vector3d current_vector(current_key.x(), current_key.y(), current_key.z());
                int key_instance = (*extended_octomap_map)[key].getInstance();
                

                // Get the search neighboorhood
                octomap::KeySet search_neighboorhood;
                for (int dx = -search_neighboorhood_ray; dx <= search_neighboorhood_ray; ++dx) {
                    for (int dy = -search_neighboorhood_ray; dy <= search_neighboorhood_ray; ++dy) {
                        for (int dz = -search_neighboorhood_ray; dz <= search_neighboorhood_ray; ++dz) {
                            // Skip the current voxel
                            if (dx == 0 && dy == 0 && dz == 0) continue;

                            octomap::OcTreeKey adjKey(key.k[0] + dx, key.k[1] + dy, key.k[2] + dz);
                            if (m_octree->search(adjKey)) { // Check if the adjacent voxel exists
                                search_neighboorhood.insert(adjKey);
                            }
                        }
                    }
                }



                // Get number of search neighboorhood keys with same key instance
                int searchCurrentInstances = 0; // Count of adjacent voxels with the same instance as the considered voxel

                // Iterate through the noise keys to count how many have the same instance
                for (const auto& adjKey : search_neighboorhood) {
                    if ((*extended_octomap_map)[adjKey].getInstance() == key_instance) {
                        searchCurrentInstances++;
                    }
                }




               
                // Calculate the threshold value (number of voxels * threshold percentage)
                int thresholdValue = static_cast<int>(search_neighboorhood.size() * threshold);


                // Check if the number of matching instances is less than the threshold value
                // In this case the voxel is an intruder
                if (searchCurrentInstances < thresholdValue) {
                    // Get the correction neighboorhood
                    octomap::KeySet correction_neighboorhood;
                    for (int dx = -correction_neighboorhood_ray; dx <= correction_neighboorhood_ray; ++dx) {
                        for (int dy = -correction_neighboorhood_ray; dy <= correction_neighboorhood_ray; ++dy) {
                            for (int dz = -correction_neighboorhood_ray; dz <= correction_neighboorhood_ray; ++dz) {
                                // Skip the current voxel
                                if (dx == 0 && dy == 0 && dz == 0) continue;

                                octomap::OcTreeKey adjKey(key.k[0] + dx, key.k[1] + dy, key.k[2] + dz);
                                if (m_octree->search(adjKey)) { // Check if the adjacent voxel exists
                                    correction_neighboorhood.insert(adjKey);
                                }
                            }
                        }
                    }

                    // Find the most frequent instance among the correction voxels
                    int mostFrequentInstance = findMostFrequentInstance(*extended_octomap_map, correction_neighboorhood);


                    if (key_instance != mostFrequentInstance) {

                        for (auto it = extended_octomap_map->begin(); it != extended_octomap_map->end(); ++it) {
                            ExtendedOctomapData& data = it->second;
                            if (data.getInstance() == mostFrequentInstance) {
                                // Set the confidence and semantic class based on the first match
                                // RCLCPP_ERROR(this->get_logger(), "conf of neighboors: %f", data.getConfidence());
                                // RCLCPP_ERROR(this->get_logger(), "color of neightboor: %f %f %f", data.confidence_r, data.confidence_g, data.confidence_b);
                                // RCLCPP_ERROR(this->get_logger(), "old conf of key: %f", (*extended_octomap_map)[key].getConfidence());
                                // RCLCPP_ERROR(this->get_logger(), "old color of key: %f %f %f", (*extended_octomap_map)[key].confidence_r, (*extended_octomap_map)[key].confidence_g, (*extended_octomap_map)[key].confidence_b);
                                (*extended_octomap_map)[key].setConfidenceNoColor(data.getConfidence());
                                (*extended_octomap_map)[key].setManualConfidenceColor(data.confidence_r, data.confidence_g, data.confidence_b);
                                (*extended_octomap_map)[key].setSemanticClass(data.getSemanticClass(), colorMap);
                                // RCLCPP_ERROR(this->get_logger(), "new conf of key: %f", (*extended_octomap_map)[key].getConfidence());
                                // RCLCPP_ERROR(this->get_logger(), "new color of key: %f %f %f", (*extended_octomap_map)[key].confidence_r, (*extended_octomap_map)[key].confidence_g, (*extended_octomap_map)[key].confidence_b);
                                break; // Exit the loop after setting the values
                            }
                        }
                        (*extended_octomap_map)[key].setInstance(mostFrequentInstance);
                    }
                    
                }

            }
            RCLCPP_INFO(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudsArrayCallback] Outlier detection done.");
        }



        if (weighted_confidence){
            // Map to store all confidences for each instance with their counts
            std::map<int, std::map<float, int>> instanceConfidencesCount;

            // Iterate over the map to group confidences by instance, excluding instance 0
            for (auto it = extended_octomap_map->begin(); it != extended_octomap_map->end(); ++it) {
                int instance = it->second.getInstance();
                if (instance == 0) continue; // Skip instance 0
                float confidence = it->second.getConfidence();
                instanceConfidencesCount[instance][confidence]++;
            }

            std::map<int, float> instanceToWeightedConfidence; // Map to store the weighted average confidence for each instance

            // Calculate weighted average for each instance
            for (auto& instanceConfidences : instanceConfidencesCount) {
                int instance = instanceConfidences.first;
                auto& confidencesCount = instanceConfidences.second;

                float sumConfidences = 0;
                float totalWeight = 0;
                for (auto& confidenceCount : confidencesCount) {
                    float confidence = confidenceCount.first;
                    int count = confidenceCount.second;
                    // Adjusting the weight to consider only the count of the confidence value
                    sumConfidences += confidence * count; // Confidence value is still used for the sum
                    totalWeight += count; // Only the count is used as weight
                }

                float weightedAverage = totalWeight > 0 ? sumConfidences / totalWeight : 0;
                instanceToWeightedConfidence[instance] = weightedAverage;
            }

            // Update the map with the new weighted average confidences
            for (auto it = extended_octomap_map->begin(); it != extended_octomap_map->end(); ++it) {
                int instance = it->second.getInstance();
                if (instanceToWeightedConfidence.find(instance) != instanceToWeightedConfidence.end()) {
                    float newConfidence = instanceToWeightedConfidence[instance];
                    it->second.setConfidenceNoColor(newConfidence);
                    it->second.setHeatConfidenceColor(newConfidence);
                }
            }
            RCLCPP_INFO(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudsArrayCallback] Weighted average done.");

        }
    




        
        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSegmentedPointcloudsArrayCallback] Extended octomap map updated with segmented pointclouds array.");


        if (publishConfidence){        
            publishConfidenceMarkers(segmented_pointclouds_array->header.stamp);
        }

        if (publishSemantic){
            publishSemanticClassMarkers(segmented_pointclouds_array->header.stamp);
        }

        if (publishInstances){
            publishInstancesMarkers(segmented_pointclouds_array->header.stamp);
        }        
    }



    /**
     * @brief Publishes all relevant occupancy OctoMap data including markers, point clouds, and maps.
     * 
     * This function is responsible for publishing various representations of the OctoMap data, such as marker arrays for 
     * occupied and free nodes, point clouds, and binary and full maps. It checks the subscription counts for different 
     * topics to determine which data needs to be published and prepares the data accordingly.
     * 
     * 
     * @param rostime The current ROS time to be used for the headers of the published messages.
     */
    void ExtendedOctomapServer::publishAll(
        const rclcpp::Time &rostime) {

        // ros::WallTime startTime = ros::WallTime::now();
        
        // total number of nodes in the tree
        size_t octomap_size = m_octree->size();
        // TODO: estimate num occ. voxels for size of arrays (reserve)
        if (octomap_size <= 1) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "Nothing to publish, octree is empty");
            return;
        }

        bool publishMarkerArray = m_markerPub->get_subscription_count() > 0;
        bool publishFreeMarkerArray = false;
        bool publishPointCloud = false;
        bool publishBinaryMap = false;
        bool publishFullMap = false;
        m_publish2DMap = false;
        if (publishFreeCells){
            publishFreeMarkerArray = m_publishFreeSpace && m_fmarkerPub->get_subscription_count()  > 0;
        }
        if (publishCentersPointcloud){
            publishPointCloud = m_pointCloudPub->get_subscription_count() > 0;
        }
        if (publishOctomapBinary){
            publishBinaryMap = m_binaryMapPub->get_subscription_count() > 0;
        }
        if (publishOctomapFull){
            publishFullMap = m_fullMapPub->get_subscription_count() > 0;
        }
        if (publish2DProjectedMap){
            m_publish2DMap = m_mapPub->get_subscription_count() > 0;
        }

        // init markers for free space:
        visualization_msgs::msg::MarkerArray freeNodesVis;
        // each array stores all cubes of a different size, one for each depth level:
        freeNodesVis.markers.resize(m_treeDepth+1);

        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, 0.0);        
        geometry_msgs::msg::Pose pose;
        pose.orientation = tf2::toMsg(quaternion);

        // init markers:
        visualization_msgs::msg::MarkerArray occupiedNodesVis;
        // each array stores all cubes of a different size, one for each depth level:
        occupiedNodesVis.markers.resize(m_treeDepth+1);

        // init pointcloud:
        pcl::PointCloud<PCLPoint> pclCloud;

        // call pre-traversal hook:
        handlePreNodeTraversal(rostime); // only for 2dmap, not relevant for now
        
        // now, traverse all leafs in the tree:
        for (auto it = m_octree->begin(m_maxTreeDepth),
                 end = m_octree->end(); it != end; ++it) {
            bool inUpdateBBX = isInUpdateBBX(it);

            // call general hook:
            // handleNode(it); // empty function, not relevant
            // if (inUpdateBBX) {
            //     handleNodeInBBX(it); // empty function, not relevant
            // }

            if (m_octree->isNodeOccupied(*it)) {
                double z = it.getZ();
                double half_size = it.getSize() / 2.0;
                if (z + half_size > m_occupancyMinZ &&
                    z - half_size < m_occupancyMaxZ) {
                    double x = it.getX();
                    double y = it.getY();
#ifdef COLOR_OCTOMAP_SERVER
                    int r = it->getColor().r;
                    int g = it->getColor().g;
                    int b = it->getColor().b;
#endif

                    // Ignore speckles in the map:
                    if (m_filterSpeckles && (it.getDepth() == m_treeDepth +1) &&
                        isSpeckleNode(it.getKey())){
                        RCLCPP_INFO(this->get_logger(),
                            "Ignoring single speckle at (%f,%f,%f)", x, y, z);
                        continue;
                    } // else: current octree node is no speckle, send it out

                    
                    handleOccupiedNode(it); // it insert the node into the 2d downprojected map. Ignore for now
                    if (inUpdateBBX) {
                        handleOccupiedNodeInBBX(it); // it insert the node into the 2d downprojected map. Ignore for now
                    }

                    //create marker:
                    if (publishMarkerArray){
                        unsigned idx = it.getDepth();
                        assert(idx < occupiedNodesVis.markers.size());

                        geometry_msgs::msg::Point cubeCenter;
                        cubeCenter.x = x;
                        cubeCenter.y = y;
                        cubeCenter.z = z;

                        occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
                        if (m_useHeightMap){
                            double minX, minY, minZ, maxX, maxY, maxZ;
                            m_octree->getMetricMin(minX, minY, minZ);
                            m_octree->getMetricMax(maxX, maxY, maxZ);

                            double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
                            occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
                        }

#ifdef COLOR_OCTOMAP_SERVER
                        if (m_useColoredMap) {
                            // TODO
                            // potentially use occupancy as measure for alpha channel?
                            std_msgs::msg::ColorRGBA _color;
                            _color.r = (r / 255.);
                            _color.g = (g / 255.);
                            _color.b = (b / 255.);
                            _color.a = 1.0;                            
                            occupiedNodesVis.markers[idx].colors.push_back(_color);
                        }
#endif
                    }

                    // insert into pointcloud:
                    if (publishPointCloud) {
#ifdef COLOR_OCTOMAP_SERVER
                        PCLPoint _point = PCLPoint();
                        _point.x = x;
                        _point.y = y;
                        _point.z = z;
                        _point.r = r;
                        _point.g = g;
                        _point.b = b;
                        pclCloud.push_back(_point);
#else
                        pclCloud.push_back(PCLPoint(x, y, z));
#endif
                    }

                }
            } else {
                // node not occupied => mark as free in 2D map if unknown so far
                double z = it.getZ();
                double half_size = it.getSize() / 2.0;
                if (z + half_size > m_occupancyMinZ &&
                    z - half_size < m_occupancyMaxZ) {
                    handleFreeNode(it);
                    if (inUpdateBBX) {
                        handleFreeNodeInBBX(it);
                    }

                    if (m_publishFreeSpace) {
                        double x = it.getX();
                        double y = it.getY();

                        //create marker for free space:
                        if (publishFreeMarkerArray) {
                            unsigned idx = it.getDepth();
                            assert(idx < freeNodesVis.markers.size());

                            geometry_msgs::msg::Point cubeCenter;
                            cubeCenter.x = x;
                            cubeCenter.y = y;
                            cubeCenter.z = z;

                            freeNodesVis.markers[idx].points.push_back(cubeCenter);
                        }
                    }
                }
            }
        }

        // call post-traversal hook:
        handlePostNodeTraversal(rostime); // only for 2dmap, not relevant for now

        // finish MarkerArray:
        if (publishMarkerArray) {
            for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
                double size = m_octree->getNodeSize(i);

                occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
                occupiedNodesVis.markers[i].header.stamp = rostime;
                occupiedNodesVis.markers[i].ns = "map";
                occupiedNodesVis.markers[i].id = i;
                occupiedNodesVis.markers[i].type =
                    visualization_msgs::msg::Marker::CUBE_LIST;
                occupiedNodesVis.markers[i].scale.x = size;
                occupiedNodesVis.markers[i].scale.y = size;
                occupiedNodesVis.markers[i].scale.z = size;
                if (!m_useColoredMap)
                    occupiedNodesVis.markers[i].color = m_color;


                if (occupiedNodesVis.markers[i].points.size() > 0)
                    occupiedNodesVis.markers[i].action =
                        visualization_msgs::msg::Marker::ADD;
                else
                    occupiedNodesVis.markers[i].action =
                        visualization_msgs::msg::Marker::DELETE;
            }
            m_markerPub->publish(occupiedNodesVis);
        }

        // finish FreeMarkerArray:
        if (publishFreeMarkerArray) {
            for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){
                double size = m_octree->getNodeSize(i);

                freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
                freeNodesVis.markers[i].header.stamp = rostime;
                freeNodesVis.markers[i].ns = "map";
                freeNodesVis.markers[i].id = i;
                freeNodesVis.markers[i].type =
                    visualization_msgs::msg::Marker::CUBE_LIST;
                freeNodesVis.markers[i].scale.x = size;
                freeNodesVis.markers[i].scale.y = size;
                freeNodesVis.markers[i].scale.z = size;
                freeNodesVis.markers[i].color = m_colorFree;

                if (freeNodesVis.markers[i].points.size() > 0)
                    freeNodesVis.markers[i].action =
                        visualization_msgs::msg::Marker::ADD;
                else
                    freeNodesVis.markers[i].action =
                        visualization_msgs::msg::Marker::DELETE;
            }
            m_fmarkerPub->publish(freeNodesVis);
        }


        // finish pointcloud:
        if (publishPointCloud) {
            sensor_msgs::msg::PointCloud2 cloud;
            pcl::toROSMsg(pclCloud, cloud);
            cloud.header.frame_id = m_worldFrameId;
            cloud.header.stamp = rostime;
            m_pointCloudPub->publish(cloud);
        }

        if (publishBinaryMap) {
            publishBinaryOctoMap(rostime);
        }
        
        if (publishFullMap) {
            publishFullOctoMap(rostime);
        }

        /*
        double total_elapsed = (ros::WallTime::now() - startTime).toSec();
        ROS_DEBUG("Map publishing in OctomapServer took %f sec", total_elapsed);
        */
    }



    /**
     * @brief Publishes the markers related to the confidence of the voxels.
     * 
     * 
     * @param rostime The current ROS time to be used for the headers of the published messages.
     */
    void ExtendedOctomapServer::publishConfidenceMarkers(const rclcpp::Time &rostime) const {
        
        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][publishConfidenceMarkers] Publishing markers of confidence...");

        bool publishConfidenceMarkers = confidenceMarkerPub->get_subscription_count() > 0;
        size_t octomap_size = m_octree->size();

        // RCLCPP_INFO(this->get_logger(), "Tree size: %lu", octomap_size);


        if (octomap_size <= 1) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "[EXTENDED OCTOMAP SERVER][publishConfidenceMarkers] Nothing to publish, octree is empty");
            return;
        }

        // init markers for free space:
        visualization_msgs::msg::MarkerArray confidenceVis;
        // each array stores all cubes of a different size, one for each depth level:
        confidenceVis.markers.resize(m_treeDepth+1);


        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, 0.0);        
        geometry_msgs::msg::Pose pose;
        pose.orientation = tf2::toMsg(quaternion);

        for (auto it = (*extended_octomap_map).begin(); it != (*extended_octomap_map).end(); ++it) {
            // Extract the OctoKey from the iterator
            octomap::OcTreeKey key = it->first;

            // Convert the OctoKey back to world coordinates
            double x = m_octree->keyToCoord(key[0]);
            double y = m_octree->keyToCoord(key[1]);
            double z = m_octree->keyToCoord(key[2]);

            //create marker:
            if (publishConfidenceMarkers){

                geometry_msgs::msg::Point cubeCenter;
                cubeCenter.x = x;
                cubeCenter.y = y;
                cubeCenter.z = z;

                confidenceVis.markers[m_maxTreeDepth].points.push_back(cubeCenter);

                std_msgs::msg::ColorRGBA _color;
                _color.r = it->second.confidence_r;
                _color.g = it->second.confidence_g;
                _color.b = it->second.confidence_b;
                _color.a = it->second.confidence_a;                            
                confidenceVis.markers[m_maxTreeDepth].colors.push_back(_color);
            }   
        }


        // finish MarkerArray:
        if (publishConfidenceMarkers) {
            for (unsigned i= 0; i < confidenceVis.markers.size(); ++i){
                double size = m_octree->getNodeSize(i);

                confidenceVis.markers[i].header.frame_id = m_worldFrameId;
                confidenceVis.markers[i].header.stamp = rostime;
                confidenceVis.markers[i].ns = "map";
                confidenceVis.markers[i].id = i;
                confidenceVis.markers[i].type =
                    visualization_msgs::msg::Marker::CUBE_LIST;
                confidenceVis.markers[i].scale.x = size;
                confidenceVis.markers[i].scale.y = size;
                confidenceVis.markers[i].scale.z = size;

                if (confidenceVis.markers[i].points.size() > 0)
                    confidenceVis.markers[i].action =
                        visualization_msgs::msg::Marker::ADD;
                else
                    confidenceVis.markers[i].action =
                        visualization_msgs::msg::Marker::DELETE;
            }
            confidenceMarkerPub->publish(confidenceVis);
        }
        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][publishConfidenceMarkers] Markers of confidences published.");


    }



    /**
     * @brief Publishes the markers related to the semantic class of the voxels.
     * 
     * 
     * @param rostime The current ROS time to be used for the headers of the published messages.
     */
    void ExtendedOctomapServer::publishSemanticClassMarkers(const rclcpp::Time &rostime) const {
        
        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][publishSemanticClassMarkers] Publishing markers of classes...");

        bool publishSemanticMarkers = semanticClassMarkerPub->get_subscription_count() > 0;
        size_t octomap_size = m_octree->size();

        if (octomap_size <= 1) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "[EXTENDED OCTOMAP SERVER][publishSemanticClassMarkers] Nothing to publish, octree is empty");
            return;
        }

        // init markers for free space:
        visualization_msgs::msg::MarkerArray semanticVis;
        // each array stores all cubes of a different size, one for each depth level:
        semanticVis.markers.resize(m_treeDepth+1);

        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, 0.0);        
        geometry_msgs::msg::Pose pose;
        pose.orientation = tf2::toMsg(quaternion);

        // RCLCPP_INFO(this->get_logger(), "Max tree depth: %d", m_maxTreeDepth);


        for (auto it = (*extended_octomap_map).begin(); it != (*extended_octomap_map).end(); ++it) {
            // Extract the OctoKey from the iterator
            octomap::OcTreeKey key = it->first;

            // Convert the OctoKey back to world coordinates
            double x = m_octree->keyToCoord(key[0]);
            double y = m_octree->keyToCoord(key[1]);
            double z = m_octree->keyToCoord(key[2]);

            //create marker:
            if (publishSemanticMarkers){

                geometry_msgs::msg::Point cubeCenter;
                cubeCenter.x = x;
                cubeCenter.y = y;
                cubeCenter.z = z;

                semanticVis.markers[m_maxTreeDepth].points.push_back(cubeCenter);

                std_msgs::msg::ColorRGBA _color;
                _color.r = it->second.semantic_r;
                _color.g = it->second.semantic_g;
                _color.b = it->second.semantic_b;
                _color.a = it->second.semantic_a;                            
                semanticVis.markers[m_maxTreeDepth].colors.push_back(_color);
            }   
        }


        // finish MarkerArray:
        if (publishSemanticMarkers) {
            for (unsigned i= 0; i < semanticVis.markers.size(); ++i){
                double size = m_octree->getNodeSize(i);

                semanticVis.markers[i].header.frame_id = m_worldFrameId;
                semanticVis.markers[i].header.stamp = rostime;
                semanticVis.markers[i].ns = "map";
                semanticVis.markers[i].id = i;
                semanticVis.markers[i].type =
                    visualization_msgs::msg::Marker::CUBE_LIST;
                semanticVis.markers[i].scale.x = size;
                semanticVis.markers[i].scale.y = size;
                semanticVis.markers[i].scale.z = size;

                if (semanticVis.markers[i].points.size() > 0)
                    semanticVis.markers[i].action =
                        visualization_msgs::msg::Marker::ADD;
                else
                    semanticVis.markers[i].action =
                        visualization_msgs::msg::Marker::DELETE;
            }
            semanticClassMarkerPub->publish(semanticVis);
        }

        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][publishSemanticClassMarkers] Markers of classes published.");

    } 

    /**
     * @brief Publishes the markers related to the instance of the voxels.
     * 
     * 
     * @param rostime The current ROS time to be used for the headers of the published messages.
     */
    void ExtendedOctomapServer::publishInstancesMarkers(const rclcpp::Time &rostime) const {
        
        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][publishInstancesMarkers] Publishing markers of instances...");

        bool publishInstancesMarkers = instancesMarkerPub->get_subscription_count() > 0;
        size_t octomap_size = m_octree->size();

        // RCLCPP_INFO(this->get_logger(), "Tree size: %lu", octomap_size);


        if (octomap_size <= 1) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "[EXTENDED OCTOMAP SERVER][publishInstancesMarkers] Nothing to publish, octree is empty");
            return;
        }

        // init markers for free space:
        visualization_msgs::msg::MarkerArray instancesVis;
        // each array stores all cubes of a different size, one for each depth level:
        instancesVis.markers.resize(m_treeDepth+1);


        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, 0.0);        
        geometry_msgs::msg::Pose pose;
        pose.orientation = tf2::toMsg(quaternion);

        for (auto it = (*extended_octomap_map).begin(); it != (*extended_octomap_map).end(); ++it) {
            // Extract the OctoKey from the iterator
            octomap::OcTreeKey key = it->first;

            // Convert the OctoKey back to world coordinates
            double x = m_octree->keyToCoord(key[0]);
            double y = m_octree->keyToCoord(key[1]);
            double z = m_octree->keyToCoord(key[2]);

            //create marker:
            if (publishInstancesMarkers){

                geometry_msgs::msg::Point cubeCenter;
                cubeCenter.x = x;
                cubeCenter.y = y;
                cubeCenter.z = z;

                instancesVis.markers[m_maxTreeDepth].points.push_back(cubeCenter);

                std_msgs::msg::ColorRGBA _color;
                _color.r = it->second.instance_r;
                _color.g = it->second.instance_g;
                _color.b = it->second.instance_b;
                _color.a = it->second.instance_a;                            
                instancesVis.markers[m_maxTreeDepth].colors.push_back(_color);
            }   
        }


        // finish MarkerArray:
        if (publishInstancesMarkers) {
            for (unsigned i= 0; i < instancesVis.markers.size(); ++i){
                double size = m_octree->getNodeSize(i);

                instancesVis.markers[i].header.frame_id = m_worldFrameId;
                instancesVis.markers[i].header.stamp = rostime;
                instancesVis.markers[i].ns = "map";
                instancesVis.markers[i].id = i;
                instancesVis.markers[i].type =
                    visualization_msgs::msg::Marker::CUBE_LIST;
                instancesVis.markers[i].scale.x = size;
                instancesVis.markers[i].scale.y = size;
                instancesVis.markers[i].scale.z = size;

                if (instancesVis.markers[i].points.size() > 0)
                    instancesVis.markers[i].action =
                        visualization_msgs::msg::Marker::ADD;
                else
                    instancesVis.markers[i].action =
                        visualization_msgs::msg::Marker::DELETE;
            }
            instancesMarkerPub->publish(instancesVis);
        }
        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][publishInstancesMarkers] Markers of instances published.");


    }


    
    bool ExtendedOctomapServer::octomapBinarySrv(
        const std::shared_ptr<OctomapSrv::Request> /*req*/, // This tells the compiler that the parameter is intentionally unused.
        std::shared_ptr<OctomapSrv::Response> res) {
        // ros::WallTime startTime = ros::WallTime::now();
        RCLCPP_INFO(this->get_logger(),
                    "Sending binary map data on service request");
        res->map.header.frame_id = m_worldFrameId;
        res->map.header.stamp = this->get_clock()->now();
        if (!octomap_msgs::binaryMapToMsg(*m_octree, res->map)) {
            return false;
        }
        
        /*
        double total_elapsed = (ros::WallTime::now() - startTime).toSec();
        ROS_INFO("Binary octomap sent in %f sec", total_elapsed);
        */
        return true;
    }

    bool ExtendedOctomapServer::octomapFullSrv(
        const std::shared_ptr<OctomapSrv::Request> /*req*/, // This tells the compiler that the parameter is intentionally unused.
        std::shared_ptr<OctomapSrv::Response> res) {
        RCLCPP_INFO(this->get_logger(),
                    "Sending full map data on service request");
        res->map.header.frame_id = m_worldFrameId;
        res->map.header.stamp = this->get_clock()->now();

        if (!octomap_msgs::fullMapToMsg(*m_octree, res->map)) {
            return false;            
        }
        return true;
    }

    bool ExtendedOctomapServer::clearBBXSrv(
        const std::shared_ptr<BBXSrv::Request> req,
        std::shared_ptr<BBXSrv::Response> /*resp*/) { // This tells the compiler that the parameter is intentionally unused.
        octomap::point3d min = pointMsgToOctomap(req->min);
        octomap::point3d max = pointMsgToOctomap(req->max);

        double thresMin = m_octree->getClampingThresMin();
        for(auto it = m_octree->begin_leafs_bbx(min,max),
                end=m_octree->end_leafs_bbx(); it!= end; ++it) {
            it->setLogOdds(octomap::logodds(thresMin));
        }
        m_octree->updateInnerOccupancy();

        rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>();
        publishAll(clock->now());

        return true;
    }

    bool ExtendedOctomapServer::resetSrv(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/, // This tells the compiler that the parameter is intentionally unused.
        std::shared_ptr<std_srvs::srv::Empty::Response> /*resp*/) { // This tells the compiler that the parameter is intentionally unused.
        visualization_msgs::msg::MarkerArray occupiedNodesVis;
        occupiedNodesVis.markers.resize(m_treeDepth + 1);
        rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>();
        auto rostime = clock->now();
        
        m_octree->clear();
        // clear 2D map:
        m_gridmap.data.clear();
        m_gridmap.info.height = 0.0;
        m_gridmap.info.width = 0.0;
        m_gridmap.info.resolution = 0.0;
        m_gridmap.info.origin.position.x = 0.0;
        m_gridmap.info.origin.position.y = 0.0;

        RCLCPP_INFO(this->get_logger(), "Cleared octomap");
        publishAll(rostime);

        publishBinaryOctoMap(rostime);
        for (size_t i = 0; i < occupiedNodesVis.markers.size(); ++i){
            occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
            occupiedNodesVis.markers[i].header.stamp = rostime;
            occupiedNodesVis.markers[i].ns = "map";
            occupiedNodesVis.markers[i].id = i;
            occupiedNodesVis.markers[i].type =
                visualization_msgs::msg::Marker::CUBE_LIST;
            occupiedNodesVis.markers[i].action =
                visualization_msgs::msg::Marker::DELETE;
        }

        m_markerPub->publish(occupiedNodesVis);

        visualization_msgs::msg::MarkerArray freeNodesVis;
        freeNodesVis.markers.resize(m_treeDepth + 1);

        for (size_t i = 0; i < freeNodesVis.markers.size(); ++i) {
            freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
            freeNodesVis.markers[i].header.stamp = rostime;
            freeNodesVis.markers[i].ns = "map";
            freeNodesVis.markers[i].id = i;
            freeNodesVis.markers[i].type =
                visualization_msgs::msg::Marker::CUBE_LIST;
            freeNodesVis.markers[i].action =
                visualization_msgs::msg::Marker::DELETE;
        }
        m_fmarkerPub->publish(freeNodesVis);
        return true;
    }

    void ExtendedOctomapServer::publishBinaryOctoMap(
        const rclcpp::Time& rostime) const{

        octomap_msgs::msg::Octomap map;
        map.header.frame_id = m_worldFrameId;
        map.header.stamp = rostime;

        if (octomap_msgs::binaryMapToMsg(*m_octree, map)) {
            m_binaryMapPub->publish(map);
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Error serializing OctoMap");
        }
    }

    void ExtendedOctomapServer::publishFullOctoMap(
        const rclcpp::Time& rostime) const{

        octomap_msgs::msg::Octomap map;
        map.header.frame_id = m_worldFrameId;
        map.header.stamp = rostime;

        if (octomap_msgs::fullMapToMsg(*m_octree, map)) {
            m_fullMapPub->publish(map);
        } else {            
            RCLCPP_ERROR(this->get_logger(),
                         "Error serializing OctoMap");
        }
    }

    void ExtendedOctomapServer::filterGroundPlane(
        const PCLPointCloud& pc,
        PCLPointCloud& ground,
        PCLPointCloud& nonground) const {
        ground.header = pc.header;
        nonground.header = pc.header;

        if (pc.size() < 50){
            RCLCPP_DEBUG(this->get_logger(),
                "Pointcloud in OctomapServer too small, skipping ground plane extraction");
            nonground = pc;
        } else {
            // plane detection for ground plane removal:
            pcl::ModelCoefficients::Ptr coefficients(
                new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(
                new pcl::PointIndices);

            // Create the segmentation object and set up:
            pcl::SACSegmentation<PCLPoint> seg;
            seg.setOptimizeCoefficients (true);
            // TODO:
            // maybe a filtering based on the surface normals might be more robust / accurate?
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(200);
            seg.setDistanceThreshold (m_groundFilterDistance);
            seg.setAxis(Eigen::Vector3f(0, 0, 1));
            seg.setEpsAngle(m_groundFilterAngle);

            PCLPointCloud cloud_filtered(pc);
            // Create the filtering object
            pcl::ExtractIndices<PCLPoint> extract;
            bool groundPlaneFound = false;

            while (cloud_filtered.size() > 10 && !groundPlaneFound) {
                seg.setInputCloud(cloud_filtered.makeShared());
                seg.segment (*inliers, *coefficients);
                if (inliers->indices.size () == 0) {
                    RCLCPP_DEBUG(this->get_logger(),
                                "PCL segmentation did not find any plane.");                    
                    break;
                }

                extract.setInputCloud(cloud_filtered.makeShared());
                extract.setIndices(inliers);

                if (std::abs(coefficients->values.at(3)) < m_groundFilterPlaneDistance) {
                    RCLCPP_DEBUG(
                        this->get_logger(),
                        "Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f",
                        inliers->indices.size(), cloud_filtered.size(),
                        coefficients->values.at(0), coefficients->values.at(1),
                        coefficients->values.at(2), coefficients->values.at(3));
                    extract.setNegative(false);
                    extract.filter(ground);

                    // remove ground points from full pointcloud:
                    // workaround for PCL bug:
                    if(inliers->indices.size() != cloud_filtered.size()) {
                        extract.setNegative(true);
                        PCLPointCloud cloud_out;
                        extract.filter(cloud_out);
                        nonground += cloud_out;
                        cloud_filtered = cloud_out;
                    }
                    groundPlaneFound = true;
                } else {
                    RCLCPP_DEBUG(
                        this->get_logger(),
                        "Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f",
                                inliers->indices.size(), cloud_filtered.size(),
                              coefficients->values.at(0), coefficients->values.at(1),
                                coefficients->values.at(2), coefficients->values.at(3));
                    pcl::PointCloud<PCLPoint> cloud_out;
                    extract.setNegative (false);
                    extract.filter(cloud_out);
                    nonground +=cloud_out;
   
                    // remove current plane from scan for next iteration:
                    // workaround for PCL bug:
                    if(inliers->indices.size() != cloud_filtered.size()){
                        extract.setNegative(true);
                        cloud_out.points.clear();
                        extract.filter(cloud_out);
                        cloud_filtered = cloud_out;
                    } else{
                        cloud_filtered.points.clear();
                    }
                }
            }
            // TODO: also do this if overall starting pointcloud too small?
            if (!groundPlaneFound){ // no plane found or remaining points too small
                RCLCPP_WARN(this->get_logger(),
                            "No ground plane found in scan");

                // do a rough fitlering on height to prevent spurious obstacles
                pcl::PassThrough<PCLPoint> second_pass;
                second_pass.setFilterFieldName("z");
                second_pass.setFilterLimits(-m_groundFilterPlaneDistance,
                                            m_groundFilterPlaneDistance);
                second_pass.setInputCloud(pc.makeShared());
                second_pass.filter(ground);

                second_pass.setNegative(true);
                second_pass.filter(nonground);
            }
        }
    }

    void ExtendedOctomapServer::handlePreNodeTraversal(
        const rclcpp::Time& rostime) {
        if (m_publish2DMap){
            // init projected 2D map:
            m_gridmap.header.frame_id = m_worldFrameId;
            m_gridmap.header.stamp = rostime;
            nav_msgs::msg::MapMetaData oldMapInfo = m_gridmap.info;

            // TODO:
            // move most of this stuff into c'tor and init map only once(adjust if size changes)
            double minX, minY, minZ, maxX, maxY, maxZ;
            m_octree->getMetricMin(minX, minY, minZ);
            m_octree->getMetricMax(maxX, maxY, maxZ);

            octomap::point3d minPt(minX, minY, minZ);
            octomap::point3d maxPt(maxX, maxY, maxZ);
            octomap::OcTreeKey minKey = m_octree->coordToKey(minPt, m_maxTreeDepth);
            octomap::OcTreeKey maxKey = m_octree->coordToKey(maxPt, m_maxTreeDepth);

            RCLCPP_INFO(
                this->get_logger(),
                "MinKey: %d %d %d / MaxKey: %d %d %d",
                minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

            // add padding if requested (= new min/maxPts in x&y):
            double halfPaddedX = 0.5*m_minSizeX;
            double halfPaddedY = 0.5*m_minSizeY;
            minX = std::min(minX, -halfPaddedX);
            maxX = std::max(maxX, halfPaddedX);
            minY = std::min(minY, -halfPaddedY);
            maxY = std::max(maxY, halfPaddedY);
            minPt = octomap::point3d(minX, minY, minZ);
            maxPt = octomap::point3d(maxX, maxY, maxZ);

            octomap::OcTreeKey paddedMaxKey;
            if (!m_octree->coordToKeyChecked(
                    minPt, m_maxTreeDepth, m_paddedMinKey)) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Could not create padded min OcTree key at %f %f %f",
                    minPt.x(), minPt.y(), minPt.z());
                return;
            }
            if (!m_octree->coordToKeyChecked(
                    maxPt, m_maxTreeDepth, paddedMaxKey)) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Could not create padded max OcTree key at %f %f %f",
                    maxPt.x(), maxPt.y(), maxPt.z());
                return;
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Padded MinKey: %d %d %d / padded MaxKey: %d %d %d",
                m_paddedMinKey[0], m_paddedMinKey[1], m_paddedMinKey[2],
                paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
            assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);
            
            m_multires2DScale = 1 << (m_treeDepth - m_maxTreeDepth);
            m_gridmap.info.width = (paddedMaxKey[0] - m_paddedMinKey[0]) /
                m_multires2DScale + 1;
            m_gridmap.info.height = (paddedMaxKey[1] - m_paddedMinKey[1]) /
                m_multires2DScale + 1;

            int mapOriginX = minKey[0] - m_paddedMinKey[0];
            int mapOriginY = minKey[1] - m_paddedMinKey[1];
            assert(mapOriginX >= 0 && mapOriginY >= 0);

            // might not exactly be min / max of octree:
            octomap::point3d origin = m_octree->keyToCoord(m_paddedMinKey, m_treeDepth);
            double gridRes = m_octree->getNodeSize(m_maxTreeDepth);
            m_projectCompleteMap = (!m_incrementalUpdate ||
                                    (std::abs(gridRes-m_gridmap.info.resolution) > 1e-6));
            
            m_gridmap.info.resolution = gridRes;
            m_gridmap.info.origin.position.x = origin.x() - gridRes*0.5;
            m_gridmap.info.origin.position.y = origin.y() - gridRes*0.5;
            if (m_maxTreeDepth != m_treeDepth) {
                m_gridmap.info.origin.position.x -= m_res/2.0;
                m_gridmap.info.origin.position.y -= m_res/2.0;
            }

            // workaround for  multires. projection not working properly for inner nodes:
            // force re-building complete map
            if (m_maxTreeDepth < m_treeDepth) {
                m_projectCompleteMap = true;
            }

            if(m_projectCompleteMap) {
                RCLCPP_INFO(this->get_logger(), "Rebuilding complete 2D map");
                m_gridmap.data.clear();
                // init to unknown:
                m_gridmap.data.resize(m_gridmap.info.width * m_gridmap.info.height, -1);

            } else {
                if (mapChanged(oldMapInfo, m_gridmap.info)){
                    RCLCPP_INFO(
                        this->get_logger(),
                        "2D grid map size changed to %dx%d",
                        m_gridmap.info.width, m_gridmap.info.height);
                    adjustMapData(m_gridmap, oldMapInfo);
                }
                nav_msgs::msg::OccupancyGrid::_data_type::iterator startIt;
                auto mapUpdateBBXMinX = std::max(
                    0, (int(m_updateBBXMin[0]) - int(m_paddedMinKey[0])) /
                    int(m_multires2DScale));
                auto mapUpdateBBXMinY = std::max(
                    0, (int(m_updateBBXMin[1]) - int(m_paddedMinKey[1])) /
                    int(m_multires2DScale));
                auto mapUpdateBBXMaxX = std::min(
                    int(m_gridmap.info.width-1),
                    (int(m_updateBBXMax[0]) - int(m_paddedMinKey[0])) /
                    int(m_multires2DScale));
                auto mapUpdateBBXMaxY = std::min(
                    int(m_gridmap.info.height-1),
                    (int(m_updateBBXMax[1]) - int(m_paddedMinKey[1])) /
                    int(m_multires2DScale));

                assert(mapUpdateBBXMaxX > mapUpdateBBXMinX);
                assert(mapUpdateBBXMaxY > mapUpdateBBXMinY);

                auto numCols = mapUpdateBBXMaxX-mapUpdateBBXMinX +1;

                // test for max idx:
                auto max_idx = m_gridmap.info.width*mapUpdateBBXMaxY + mapUpdateBBXMaxX;
                if (max_idx  >= m_gridmap.data.size()) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        std::string("BBX index not valid:").c_str(),
                        "%d (max index %zu for size %d x %d) update-BBX is: ",
                        "[%zu %zu]-[%zu %zu]",
                        max_idx, m_gridmap.data.size(), m_gridmap.info.width,
                        m_gridmap.info.height, mapUpdateBBXMinX,
                        mapUpdateBBXMinY, mapUpdateBBXMaxX, mapUpdateBBXMaxY);
                }

                // reset proj. 2D map in bounding box:
                for (unsigned int j = mapUpdateBBXMinY; j <= static_cast<unsigned int>(mapUpdateBBXMaxY); ++j) {
                    std::fill_n(
                        m_gridmap.data.begin() + m_gridmap.info.width*j+mapUpdateBBXMinX,
                        numCols, -1);
                }
            }
        }
    }

    void ExtendedOctomapServer::handlePostNodeTraversal(
        const rclcpp::Time& /*rostime*/){ // This tells the compiler that the parameter is intentionally unused.
        if (m_publish2DMap) {
            m_mapPub->publish(m_gridmap);
        }
    }

    void ExtendedOctomapServer::handleOccupiedNode(
        const OcTreeT::iterator& it){
        if (m_publish2DMap && m_projectCompleteMap){
            update2DMap(it, true);
        }
    }

    void ExtendedOctomapServer::handleFreeNode(
        const OcTreeT::iterator& it){
        if (m_publish2DMap && m_projectCompleteMap){
            update2DMap(it, false);
        }
    }

    void ExtendedOctomapServer::handleOccupiedNodeInBBX(
        const OcTreeT::iterator& it){
        if (m_publish2DMap && !m_projectCompleteMap){
            update2DMap(it, true);
        }
    }

    void ExtendedOctomapServer::handleFreeNodeInBBX(
        const OcTreeT::iterator& it){
        if (m_publish2DMap && !m_projectCompleteMap){
            update2DMap(it, false);
        }
    }

    void ExtendedOctomapServer::update2DMap(
        const OcTreeT::iterator& it, bool occupied) {
        if (it.getDepth() == m_maxTreeDepth){
            auto idx = mapIdx(it.getKey());
            if (occupied) {
                m_gridmap.data[mapIdx(it.getKey())] = 100;
            } else if (m_gridmap.data[idx] == -1){
                m_gridmap.data[idx] = 0;
            }
        } else {
            int intSize = 1 << (m_maxTreeDepth - it.getDepth());
            octomap::OcTreeKey minKey=it.getIndexKey();
            for(int dx = 0; dx < intSize; dx++) {
                int i = (minKey[0] + dx - m_paddedMinKey[0]) / m_multires2DScale;
                for(int dy = 0; dy < intSize; dy++){
                    auto idx = mapIdx(i, (minKey[1] + dy - m_paddedMinKey[1]) /
                                      m_multires2DScale);
                    if (occupied) {
                        m_gridmap.data[idx] = 100;
                    } else if (m_gridmap.data[idx] == -1) {
                        m_gridmap.data[idx] = 0;
                    }
                }
            }
        }
    }

    bool ExtendedOctomapServer::isSpeckleNode(
        const octomap::OcTreeKey &nKey) const {
        octomap::OcTreeKey key;
        bool neighborFound = false;
        for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]) {
            for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]) {
                for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]) {
                    if (key != nKey) {
                        auto node = m_octree->search(key);
                        if (node && m_octree->isNodeOccupied(node)) {
                            neighborFound = true;
                        }
                    }
                }
            }
        }
        return neighborFound;
    }

    void ExtendedOctomapServer::adjustMapData(
        nav_msgs::msg::OccupancyGrid &map,
        const nav_msgs::msg::MapMetaData &oldMapInfo) const {
        if (map.info.resolution != oldMapInfo.resolution) {
            RCLCPP_ERROR(this->get_logger(),
                "Resolution of map changed, cannot be adjusted");
            return;
        }

        int i_off = int(
            (oldMapInfo.origin.position.x - map.info.origin.position.x) /
            map.info.resolution + 0.5);
        int j_off = int(
            (oldMapInfo.origin.position.y - map.info.origin.position.y) /
            map.info.resolution + 0.5);

        if (i_off < 0 || j_off < 0
            || oldMapInfo.width  + i_off > map.info.width
            || oldMapInfo.height + j_off > map.info.height) {
            RCLCPP_ERROR(
                this->get_logger(),
                "New 2D map does not contain old map area, this case is not implemented");
            return;
        }

        // nav_msgs::msg::OccupancyGrid::_data_type oldMapData =
        // map.data;
        auto oldMapData = map.data;

        map.data.clear();
        // init to unknown:
        map.data.resize(map.info.width * map.info.height, -1);
        nav_msgs::msg::OccupancyGrid::_data_type::iterator fromStart, fromEnd, toStart;

        for (int j = 0; j < int(oldMapInfo.height); ++j) {
            // copy chunks, row by row:
            fromStart = oldMapData.begin() + j*oldMapInfo.width;
            fromEnd = fromStart + oldMapInfo.width;
            toStart = map.data.begin() + ((j+j_off)*m_gridmap.info.width + i_off);
            copy(fromStart, fromEnd, toStart);
        }
    }

    std_msgs::msg::ColorRGBA ExtendedOctomapServer::heightMapColor(double h) {
        std_msgs::msg::ColorRGBA color;
        color.a = 1.0;
        // blend over HSV-values (more colors)

        double s = 1.0;
        double v = 1.0;

        h -= floor(h);
        h *= 6;
        int i;
        double m, n, f;

        i = floor(h);
        f = h - i;
        if (!(i & 1))
            f = 1 - f; // if i is even
        m = v * (1 - s);
        n = v * (1 - s * f);

        switch (i) {
        case 6:
        case 0:
            color.r = v; color.g = n; color.b = m;
            break;
        case 1:
            color.r = n; color.g = v; color.b = m;
            break;
        case 2:
            color.r = m; color.g = v; color.b = n;
            break;
        case 3:
            color.r = m; color.g = n; color.b = v;
            break;
        case 4:
            color.r = n; color.g = m; color.b = v;
            break;
        case 5:
            color.r = v; color.g = m; color.b = n;
            break;
        default:
            color.r = 1; color.g = 0.5; color.b = 0.5;
            break;
        }
        return color;
    }




    void ExtendedOctomapServer::setInsertCloudActive(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        (void)request_header;
        insertCloudActive = request->data;
        response->success = true;

        if (request->data){
            response->message = "Insert cloud callback activated successfully";
        }
        else {
            response->message = "Insert cloud callback deactivated successfully";
        }
    }

    void ExtendedOctomapServer::setinsertSegmentedActive(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        (void)request_header;
        insertSegmentedActive = request->data;
        response->success = true;
        
        if (request->data){
            response->message = "Insert segmented callback activated successfully";
        }
        else {
            response->message = "Insert segmented callback deactivated successfully";
        }
    }   



    /**
     * @brief Finds the most frequent instance in the given set of point cloud keys.
     * 
     * This function iterates over the provided set of point cloud keys, retrieves the instance associated with each key,
     * and counts the frequency of each instance. It then determines the most frequent instance. If the useFrequencyThreshold
     * parameter is enabled, the function handles cases where the most frequent instance is 0 by checking if the second most
     * frequent instance meets a specified threshold.
     * 
     * 
     * @param map The extended OctoMap containing the voxels.
     * @param pointcloudKeys The set of point cloud keys to analyze.
     * @return The instance (key) of the most frequent element.
     */
    int ExtendedOctomapServer::findMostFrequentInstance(ExtendedOctomapMap& map, const octomap::KeySet& pointcloudKeys) {
        std::map<int, int> instanceFrequency; // Map to store frequency of each instance

        // Iterate over each key in the KeySet
        for (const auto& key : pointcloudKeys) {
            int instance = map[key].getInstance(); // Get the object from the key
            instanceFrequency[instance]++; // Increment the frequency count for this instance
        }

        // Print the frequency of each instance
        for (const auto& freq : instanceFrequency) {
            RCLCPP_DEBUG(this->get_logger(), "Instance %d appears %d times", freq.first, freq.second);
        }

        // Find the instance with the highest frequency
        auto mostFrequent = std::max_element(instanceFrequency.begin(), instanceFrequency.end(),
            [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                return a.second < b.second;
            });
        
        if (useFrequencyThreshold){
            if (mostFrequent->first == 0){
               // Remove the most frequent instance if it is 0
                instanceFrequency.erase(mostFrequent);

                // Find the new most frequent instance, which is now the second most frequent from the original map
                if (!instanceFrequency.empty()) {
                    auto secondMostFrequent = std::max_element(instanceFrequency.begin(), instanceFrequency.end(),
                        [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                            return a.second < b.second;
                        });
                    // Check if second instance is at least %threshold of the instnce 0. If yes, that is the most frequent
                    if (mostFrequent->second * frequencyThreshold < secondMostFrequent->second){
                        return secondMostFrequent->first;
                    }
                    else {
                        return mostFrequent->first;
                    }
                } else {
                    // Handle the case where all instances were 0 or the map is empty
                    return mostFrequent->first; // return 0
                } 
            }
            else{
                return mostFrequent->first; // Return the instance (key) of the most frequent element
            }
        }
        else {
            return mostFrequent->first; // Return the instance (key) of the most frequent element
        }
    }
    
    
    /**
     * @brief Counts the number of points in a voxel.
     * 
     * This function iterates over the provided point cloud and counts the number of points that fall within the specified voxel.
     * It converts each point in the point cloud to an OcTree key and compares it with the target key. If they match, the count is incremented.
     * 
     * 
     * @param pointcloud The point cloud to analyze.
     * @param targetKey The OcTree key of the target voxel.
     * @param m_octree The shared pointer to the OcTree.
     * @return The number of points in the specified voxel.
     */
    int ExtendedOctomapServer::countPointsInVoxel(const PCLPointCloud& pointcloud, const octomap::OcTreeKey& targetKey, std::shared_ptr<OcTreeT> m_octree) {
        int count = 0;
        for (const auto& point : pointcloud) {
            octomap::point3d point_3d(point.x, point.y, point.z);                
            octomap::OcTreeKey key;
            if (m_octree->coordToKeyChecked(point_3d, key)) {
                if (key == targetKey) {
                    ++count;
                }
            }
        }
        return count;
    }


    double ExtendedOctomapServer::getRes(){
        return this->m_res;
    }


    std::shared_ptr<OcTreeT> ExtendedOctomapServer::getOcTree(){
        return this->m_octree;
    }


    std::shared_ptr<ExtendedOctomapMap> ExtendedOctomapServer::getExtendedOctomapMap(){
        return this->extended_octomap_map;
    }



    /**
     * @brief Saves the OctoMap to a binary file.
     * 
     * This function saves the current state of the OctoMap to a specified binary file. It checks if the OctoMap is initialized,
     * opens the file for writing, and writes the OctoMap data in binary format.
     * 
     * @param filename The name of the file to save the OctoMap to.
     * @return True if the OctoMap was successfully saved, false otherwise.
     */
    bool ExtendedOctomapServer::saveOctree(const std::string &filename) {
        if (!m_octree) {
            RCLCPP_ERROR(this->get_logger(), "Octree is not initialized");
            return false;
        }
        std::ofstream ofs(filename, std::ios::binary);
        if (!ofs) {
            RCLCPP_ERROR(this->get_logger(), "Could not open file for writing: %s", filename.c_str());
            return false;
        }
        m_octree->writeBinary(ofs);
        ofs.close();
        return true;
    }


    /**
     * @brief Loads the OctoMap from a binary file.
     * 
     * This function loads the state of the OctoMap from a specified binary file. It opens the file for reading, creates a new 
     * OctoMap object, and reads the OctoMap data from the file in binary format.
     * 
     * @param filename The name of the file to load the OctoMap from.
     * @return A shared pointer to the loaded OctoMap object, or nullptr if the file could not be opened.
     */
    std::shared_ptr<OcTreeT> ExtendedOctomapServer::loadOctree(const std::string &filename) {
        std::ifstream ifs(filename, std::ios::binary);
        if (!ifs) {
            RCLCPP_ERROR(this->get_logger(), "Could not open file for reading: %s", filename.c_str());
            return nullptr;
        }
        auto m_octree = std::make_shared<OcTreeT>(m_res);
        m_octree->readBinary(ifs);
        ifs.close();
        return m_octree;
    }

    /**
     * @brief Callback for saving OctoMap data via a service request.
     * 
     * This function handles service requests to save the current state of the OctoMap. If the request data is true, it performs
     * the save operation by generating a filename based on the current time, saving the OctoMap to a binary file, and setting
     * the response message accordingly.
     * 
     * @param request The service request containing the data to process.
     * @param response The service response to be populated with the result of the operation.
     */
    void ExtendedOctomapServer::saveOctomapDataServiceCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data) { // If the request is true, perform the save and load operations

            // Get the current time
            std::time_t now = std::time(nullptr);
            char buf[16]; // Buffer to hold the formatted time
            // Format the time - Note: Year %y is two digits, for four digits use %Y
            std::strftime(buf, sizeof(buf), "%y%m%d%H%M%S", std::localtime(&now));

            // Create the filename
            std::stringstream tree_filename;
            std::string package_path = ament_index_cpp::get_package_share_directory("av_bringup");
            tree_filename << package_path << "/data/octree_" << buf << "_truth.bt";
            
            bool tree_save = saveOctree(tree_filename.str());

            if (tree_save) {
                response->success = true;
                response->message = "Data saved.";
            } else {
                response->success = false;
                response->message = "Failed to save the data";
            }
        } else {
            response->success = false;
            response->message = "Request data is false, not processing.";
        }
    }







    // Transforms
    void ExtendedOctomapServer::transformAsMatrix(const tf2::Transform &bt,
                           Eigen::Matrix4f &out_mat) {
        double mv[12];
        bt.getBasis ().getOpenGLSubMatrix (mv);

        tf2::Vector3 origin = bt.getOrigin ();

        out_mat (0, 0) = mv[0]; out_mat (0, 1) = mv[4]; out_mat (0, 2) = mv[8];
        out_mat (1, 0) = mv[1]; out_mat (1, 1) = mv[5]; out_mat (1, 2) = mv[9];
        out_mat (2, 0) = mv[2]; out_mat (2, 1) = mv[6]; out_mat (2, 2) = mv[10];
                                                                     
        out_mat (3, 0) = out_mat(3, 1) = out_mat(3, 2) = 0;
        out_mat(3, 3) = 1;
        out_mat (0, 3) = origin.x();
        out_mat (1, 3) = origin.y();
        out_mat (2, 3) = origin.z();
    }

    Eigen::Matrix4f ExtendedOctomapServer::transformAsMatrix(
        const geometry_msgs::msg::TransformStamped &transform_stamped) {

        auto t = transform_stamped.transform.translation;        
        auto r = transform_stamped.transform.rotation;
        tf2::Quaternion quaternion(r.x, r.y, r.z, r.w);
        tf2::Vector3 translation(t.x, t.y, t.z);
        
        tf2::Transform transform;
        transform.setOrigin(translation);
        transform.setRotation(quaternion);

        Eigen::Matrix4f out_mat;
        ExtendedOctomapServer::transformAsMatrix(transform, out_mat);
        return out_mat;        
    }


    // Conversions
    void ExtendedOctomapServer::pointsOctomapToPointCloud2(const octomap::point3d_list& /*points*/,
                                    sensor_msgs::msg::PointCloud2& cloud) {
        // make sure the channel is valid

        /*
        std::vector<sensor_msgs::PointField>::const_iterator field_iter =
            cloud.fields.begin(),
            field_end = cloud.fields.end();
        */

        auto field_iter = cloud.fields.begin();
        auto field_end = cloud.fields.end();
        
        bool has_x, has_y, has_z;
        has_x = has_y = has_z = false;
        while (field_iter != field_end) {
            if ((field_iter->name == "x") || (field_iter->name == "X"))
                has_x = true;
            if ((field_iter->name == "y") || (field_iter->name == "Y"))
                has_y = true;
            if ((field_iter->name == "z") || (field_iter->name == "Z"))
                has_z = true;
            ++field_iter;
        }

        if ((!has_x) || (!has_y) || (!has_z))
            throw std::runtime_error("One of the fields xyz does not exist");

        /*
        sensor_msgs::msg::PointCloud2Modifier pcd_modifier(cloud);
        pcd_modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (point3d_list::const_iterator it = points.begin(); it != points.end();
             ++it, ++iter_x, ++iter_y, ++iter_z) {
            *iter_x = it->x();
            *iter_y = it->y();
            *iter_z = it->z();
        }
        */
    }

    /**
     * @brief Conversion from a sensor_msgs::PointCLoud2 to
     * octomap::Pointcloud,
     used internally in OctoMap
     *
     * @param cloud
     * @param octomapCloud
     */

    void ExtendedOctomapServer::pointCloud2ToOctomap(const sensor_msgs::msg::PointCloud2 &cloud,
                              octomap::Pointcloud &octomapCloud){
        octomapCloud.reserve(cloud.data.size() / cloud.point_step);

        /*
          sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
          sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
          sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

          for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z){
          // Check if the point is invalid
          if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
          octomapCloud.push_back(*iter_x, *iter_y, *iter_z);
          }
        */
    }

}

