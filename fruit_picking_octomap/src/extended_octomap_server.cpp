#include <fruit_picking_octomap/extended_octomap_server.hpp>

namespace extended_octomap_server {

    // Constructor
    ExtendedOctomapServer::ExtendedOctomapServer(const rclcpp::NodeOptions &options, const std::string node_name) : octomap_server::OctomapServer::OctomapServer(options, node_name) {
        
        RCLCPP_INFO(this->get_logger(), "Extended constructor started.");

        
        // Initialization of the map for the additional semantic information
        extended_octomap_map = std::make_shared<ExtendedOctomapMap>();

        // Initialization of the map to store the key of the voxel where there are collisio between points of different instances
        collisionKeys = std::make_shared<CollisionOcTreeKeys>();


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

        semanticPointcloudSubscription = this->declare_parameter(
            "semantic_pointcloud_subscription", semanticPointcloudSubscription);
        semanticPointcloudsArraySubscription = this->declare_parameter(
            "semantic_pointclouds_array_subscription", semanticPointcloudsArraySubscription);



        // Initialize the services for the activation of the callbacks
        insertCloudActive = this->declare_parameter(
            "insert_cloud_init", insertCloudActive);
        insertSemanticActive = this->declare_parameter(
            "insert_semantic_init", insertSemanticActive);

        insertCloudActiveService_ = this->create_service<std_srvs::srv::SetBool>(
            "set_insert_cloud_active",
            std::bind(&ExtendedOctomapServer::setInsertCloudActive, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        insertSemanticActiveService_ = this->create_service<std_srvs::srv::SetBool>(
            "set_insert_semantic_active",
            std::bind(&ExtendedOctomapServer::setInsertSemanticActive, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));



        // Initialization of the variable to keep track of the instances
        currentMaxInstance = 1;     


        // Initialization of the subscribers and publishers, with their callbacks
        this->onInit();
    }



    ExtendedOctomapServer::~ExtendedOctomapServer(){}



    void ExtendedOctomapServer::onInit() {
        
        RCLCPP_INFO(this->get_logger(), "Initialization of extended octomap server started...");


        if (semanticPointcloudsArraySubscription){
            this->segmentedPointcloudsArraySub = std::make_shared<
                message_filters::Subscriber<fruit_picking_interfaces::msg::PointcloudArray>>(
                    this, "segmented_pointclouds_array", rmw_qos_profile_sensor_data);
            
            this->tfSegmentedPointcloudsArraySub = std::make_shared<tf2_ros::MessageFilter<
                fruit_picking_interfaces::msg::PointcloudArray>>(
                    *buffer_, m_worldFrameId, messageFilterQueue,
                    this->get_node_logging_interface(),
                    this->get_node_clock_interface(),
                    std::chrono::seconds(1));
            this->tfSegmentedPointcloudsArraySub->connectInput(*segmentedPointcloudsArraySub);
            this->tfSegmentedPointcloudsArraySub->registerCallback(
                std::bind(&ExtendedOctomapServer::insertSemanticArrayCallback, this, ph::_1));
            
            RCLCPP_INFO(this->get_logger(), "Subscription to semantic pointclouds array topic done.");
        }

        if (semanticPointcloudSubscription){
            this->segmentedPointcloudSub = std::make_shared<
                message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
                    this, "segmented_pointcloud", rmw_qos_profile_sensor_data);
            
            this->tfSegmentedPointcloudSub = std::make_shared<tf2_ros::MessageFilter<
                sensor_msgs::msg::PointCloud2>>(
                    *buffer_, m_worldFrameId, messageFilterQueue,
                    this->get_node_logging_interface(),
                    this->get_node_clock_interface(),
                    std::chrono::seconds(1));
            this->tfSegmentedPointcloudSub->connectInput(*segmentedPointcloudSub);
            this->tfSegmentedPointcloudSub->registerCallback(
                std::bind(&ExtendedOctomapServer::insertSemanticCallback, this, ph::_1));

            RCLCPP_INFO(this->get_logger(), "Subscription to semantic pointcloud topic done.");
        }


        rclcpp::QoS qos(rclcpp::KeepLast(3));
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
    }



    void ExtendedOctomapServer::insertCloudCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud){

        if (insertCloudActive){
        
            OctomapServer::insertCloudCallback(cloud);

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
    }


    void ExtendedOctomapServer::insertScan(
        const geometry_msgs::msg::Vector3 &sensorOriginTf,
        const PCLPointCloud& ground,
        const PCLPointCloud& nonground) 
    {
        octomap::point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);
        
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

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        if (processFreeSpace){
            RCLCPP_DEBUG(this->get_logger(), "Duration of the insertion of the pointcloud in the free and occupied cells: %f", elapsed_seconds.count());
        }
        else{
            RCLCPP_DEBUG(this->get_logger(), "Duration of the insertion of the pointcloud in the occupied cells: %f", elapsed_seconds.count());
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
        
#ifdef COLOR_OCTOMAP_SERVER
        if (colors) {
            delete[] colors;
            colors = NULL;
        }
#endif
    }



    void ExtendedOctomapServer::insertSemanticCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &segmented_pointcloud){

        // If the paramter to activate the callback is false, the callback will be skipped
        if (!insertSemanticActive){
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticCallback] Semantic callback started.");

        // From pointcloud message to Pointcloud data structure
        PCLPointCloud segmented_pc;
        pcl::fromROSMsg(*segmented_pointcloud, segmented_pc);

        // Conversion of the pointcloud from sensor frame to world frame
        Eigen::Matrix4f sensorToWorld;
        geometry_msgs::msg::TransformStamped sensorToWorldTf;
        try {
            if (!this->buffer_->canTransform(
                    m_worldFrameId, segmented_pointcloud->header.frame_id,
                    segmented_pointcloud->header.stamp)) {
                throw "Failed";
            }
            
            sensorToWorldTf = this->buffer_->lookupTransform(
                m_worldFrameId, segmented_pointcloud->header.frame_id,
                segmented_pointcloud->header.stamp);
            sensorToWorld = pcl_ros::transformAsMatrix(sensorToWorldTf);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s",ex.what());
            return;
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
            RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticCallback] Pointcloud after filtering is empty, skipping to next iteration.");
            return; // Go to the next iteration of the loop
        }


        for (auto it = segmented_pc.begin(); it != segmented_pc.end(); ++it) {
            octomap::point3d point(it->x, it->y, it->z);                
            octomap::OcTreeKey key;

            if (m_octree->coordToKeyChecked(point, key)) {
                if (extended_octomap_map->find(key) != extended_octomap_map->end()) {
                    (*extended_octomap_map)[key].setSemanticClass("mask", colorMap);
                }
            }
        }

        if (publishConfidence){        
            publishConfidenceMarkers(segmented_pointcloud->header.stamp);
        }

        if (publishSemantic){
            publishSemanticClassMarkers(segmented_pointcloud->header.stamp);
        }

        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticCallback] Extended octomap map updated with segmented pointcloud.");
    }


    void ExtendedOctomapServer::insertSemanticArrayCallback(const fruit_picking_interfaces::msg::PointcloudArray::ConstSharedPtr &segmented_pointclouds_array){


        // If the paramter to activate the callback is false, the callback will be skipped
        if (!insertSemanticActive){
            return;
        }

        // The callback starts the execution
        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] Semantic array callback started.");
        

        // If the input data composed of segmented pointclouds and relative confidences is empty, terminate the callback
        if (segmented_pointclouds_array->confidences.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] Semantic array callback finished with no effect.");    
            return;   
        }


        // If it is not empty, save confidence vector
        std::vector<float> confidences;
        for (auto& kv : segmented_pointclouds_array->confidences) {
            confidences.push_back(std::stof(kv.value));
        }

        // Save input semantic class
        std::string semantic_class = segmented_pointclouds_array->semantic_class;


        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] Entering loop...");


        // Loop thorugh all the poitclouds inside the array
        for (size_t i = 0; i < segmented_pointclouds_array->pointclouds.size(); ++i){

            // Take the pointcloud and the confidence
            auto& segmented_pointcloud = segmented_pointclouds_array->pointclouds[i];
            float confidence = confidences[i];


            // Data structure to save all the octreekeys that contain the current pointcloud,
            // so that the next operation will be done iterating through this structure
            octomap::KeySet pointcloudKeys;

            

            // From pointcloud message to Pointcloud data structure
            PCLPointCloud segmented_pc;
            pcl::fromROSMsg(segmented_pointcloud, segmented_pc);

            // Conversion of the pointcloud from sensor frame to world frame
            Eigen::Matrix4f sensorToWorld;
            geometry_msgs::msg::TransformStamped sensorToWorldTf;
            try {
                if (!this->buffer_->canTransform(
                        m_worldFrameId, segmented_pointcloud.header.frame_id,
                        segmented_pointcloud.header.stamp)) {
                    throw "Failed";
                }
                
                sensorToWorldTf = this->buffer_->lookupTransform(
                    m_worldFrameId, segmented_pointcloud.header.frame_id,
                    segmented_pointcloud.header.stamp);
                sensorToWorld = pcl_ros::transformAsMatrix(sensorToWorldTf);
            } catch (tf2::TransformException &ex) {
                return;
            } catch (const std::exception& e) {
                // This will catch standard exceptions
                RCLCPP_ERROR(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] %s",e.what());
            } catch (...) {
                // This will catch all other exceptions
                RCLCPP_ERROR(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] Generic error.");
            }

            pcl::transformPointCloud(segmented_pc, segmented_pc, sensorToWorld);

            RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] Pointcloud before the filtering contains %d points", segmented_pc.size());


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
                RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] Pointcloud after filtering is empty, skipping to next iteration.");
                continue; // Go to the next iteration of the loop
            }



            // Save the octreekeys into the data structure, if it exist, skip, since more points can be in the same key
            for (auto it = segmented_pc.begin(); it != segmented_pc.end(); ++it) {
                octomap::point3d point(it->x, it->y, it->z);                
                octomap::OcTreeKey key;

                if (m_octree->coordToKeyChecked(point, key)) { // find the key of the point
                    // Try to insert the key. If the key is already present, the insertion will fail, but it's fine since we don't want duplicates.
                    pointcloudKeys.insert(key);
                }
            }

            RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] pointcloudKeys filled with %d keys belonging to the pointcloud with %d points", pointcloudKeys.size(), segmented_pc.size());

            // Save the most frequent instance value of the previous keys
            int most_frequent_instance = findMostFrequentInstance(*extended_octomap_map, pointcloudKeys);

            RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] The most frequent instance of these keys is %d", most_frequent_instance);


            
            // Variable to check if an instance has been inserted
            bool current_max_instance_used = false;

            RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] Iterating through keys...");

            // Iterate through the octreekeys related to the current pointcloud
            for(const auto& pt_key : pointcloudKeys) {
                
                // Get info of the key
                int pt_key_instance = (*extended_octomap_map)[pt_key].getInstance();
                std::string pt_key_class = (*extended_octomap_map)[pt_key].getSemanticClass();
                int pt_key_points_count = (*extended_octomap_map)[pt_key].getPointsCount();
                float pt_key_confidence = (*extended_octomap_map)[pt_key].getConfidence();

                // Case when the key has the same value as the major instance value of the entire pointcloud
                if (pt_key_instance == most_frequent_instance){
                    (*extended_octomap_map)[pt_key].setConfidenceMaxFusion(semantic_class, colorMap, confidence);
                    int pt_key_count = countPointsInVoxel(segmented_pc, pt_key, m_octree);
                    (*extended_octomap_map)[pt_key].setPointsCount(pt_key_count);

                    // if the instance of this key is zero means that need to be initializated with the current max (and free) instance value
                    if (pt_key_instance == 0){
                        (*extended_octomap_map)[pt_key].setInstance(currentMaxInstance);
                        current_max_instance_used = true; // flag to say that this instance value has been used
                    }
                }

                

                // Case when the key has not the same value as the major instance value of the entire pointcloud, and the 
                // major instance value is zero, meaning that the instance has not been initialized
                else if (pt_key_instance != most_frequent_instance && most_frequent_instance == 0){
                    int pt_key_current_count = countPointsInVoxel(segmented_pc, pt_key, m_octree); 
                    std::list<std::tuple<int, std::string, int, float>> tuplesList;
                    tuplesList.push_back(std::make_tuple(pt_key_instance, pt_key_class, pt_key_points_count, pt_key_confidence));
                    tuplesList.push_back(std::make_tuple(currentMaxInstance, semantic_class, pt_key_current_count, confidence));
                    (*collisionKeys)[pt_key] = tuplesList;
                }
            }


            // Increment the current max instance value if this variable has been used for at least one octreekey
            if (current_max_instance_used){
                currentMaxInstance++;
                RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] Instance incremented: %d", currentMaxInstance);
            }
            

            
        }

        // Check for the collisions octreekeys

        // At each iteration, the collisions of an octreekey are checked
        for (auto it = collisionKeys->begin(); it != collisionKeys->end(); ++it) {
            const auto& tuplesList = it->second;
            
            RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] Collision keys:");
            std::string logMsg = "Key: (" + std::to_string(it->first.k[0]) + ", " + std::to_string(it->first.k[1]) + ", " + std::to_string(it->first.k[2]) + ") Values: ";
            for (const auto& tuple : tuplesList) {
                logMsg += "Tuple: (";
                logMsg += std::to_string(std::get<0>(tuple)) + ", "; // int
                logMsg += std::get<1>(tuple) + ", "; // std::string
                logMsg += std::to_string(std::get<2>(tuple)) + ", "; // int
                logMsg += std::to_string(std::get<3>(tuple)); // float
                logMsg += ") ";
            }
            RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] %s", logMsg.c_str());

            
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

        
        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] Extended octomap map updated with segmented pointcloud array.");


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



    void ExtendedOctomapServer::publishConfidenceMarkers(const rclcpp::Time &rostime) const {
        
        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][publishConfidenceMarkers] Publishing markers of confidence...");

        bool publishConfidenceMarkers = confidenceMarkerPub->get_subscription_count() > 0;
        size_t octomap_size = m_octree->size();

        // RCLCPP_INFO(this->get_logger(), "Tree size: %lu", octomap_size);


        if (octomap_size <= 1) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "Nothing to publish, octree is empty");
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
            double size = m_octree->getNodeSize(m_maxTreeDepth);

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




    void ExtendedOctomapServer::publishSemanticClassMarkers(const rclcpp::Time &rostime) const {
        
        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][publishSemanticClassMarkers] Publishing markers of classes...");

        bool publishSemanticMarkers = semanticClassMarkerPub->get_subscription_count() > 0;
        size_t octomap_size = m_octree->size();

        if (octomap_size <= 1) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "Nothing to publish, octree is empty");
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
            double size = m_octree->getNodeSize(m_maxTreeDepth);

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


    void ExtendedOctomapServer::publishInstancesMarkers(const rclcpp::Time &rostime) const {
        
        RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][publishInstancesMarkers] Publishing markers of instances...");

        bool publishInstancesMarkers = instancesMarkerPub->get_subscription_count() > 0;
        size_t octomap_size = m_octree->size();

        // RCLCPP_INFO(this->get_logger(), "Tree size: %lu", octomap_size);


        if (octomap_size <= 1) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "Nothing to publish, octree is empty");
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
            double size = m_octree->getNodeSize(m_maxTreeDepth);

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

    void ExtendedOctomapServer::setInsertSemanticActive(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        (void)request_header;
        insertSemanticActive = request->data;
        response->success = true;
        
        if (request->data){
            response->message = "Insert semantic callback activated successfully";
        }
        else {
            response->message = "Insert semantic callback deactivated successfully";
        }
    }   


    // Function to find the most frequent instance of a set of octreekeys
    int ExtendedOctomapServer::findMostFrequentInstance(ExtendedOctomapMap& map, const octomap::KeySet& pointcloudKeys) {
        std::map<int, int> instanceFrequency; // Map to store frequency of each instance

        // Iterate over each key in the KeySet
        for (const auto& key : pointcloudKeys) {
            int instance = map[key].getInstance(); // Get the object from the key
            instanceFrequency[instance]++; // Increment the frequency count for this instance
        }

        // Find the instance with the highest frequency
        auto mostFrequent = std::max_element(instanceFrequency.begin(), instanceFrequency.end(),
            [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                return a.second < b.second;
            });
        
        return mostFrequent->first; // Return the instance (key) of the most frequent element
    }
    
    
    
    // Function to count the number of points inside a octree node
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

}


RCLCPP_COMPONENTS_REGISTER_NODE(extended_octomap_server::ExtendedOctomapServer)
