#include <fruit_picking_octomap/extended_octomap_server.hpp>

namespace extended_octomap_server {

    ExtendedOctomapData::ExtendedOctomapData(){
        this->semantic_class = "none";            
        this->confidence = 0.0;
        setConfidenceColor(0.0);
        setSemanticColor("none");
    }


    void ExtendedOctomapData::setSemanticClass(std::string semantic_class){
        this->semantic_class = semantic_class;
        setSemanticColor(semantic_class);
    }

    void ExtendedOctomapData::setConfidence(float confidence){
        this->confidence = confidence;
        setConfidenceColor(confidence);
    }

    void ExtendedOctomapData::setConfidenceMaxFusion(std::string semantic_class, float confidence, float penalization=0.9){
        std::string sem_1 = this->semantic_class;
        std::string sem_2 = semantic_class;
        float c_1 = this->confidence;
        float c_2 = confidence;

        std::string sem_f;
        float c_f;

        if (sem_1 == sem_2){
            sem_f = sem_2;
            c_f = (c_1 + c_2) / 2;
        }
        else {
            if (c_1 >= c_2){
                sem_f = sem_1;
                c_f = penalization * c_1;
            }
            else if (c_1 < c_2) {
                sem_f = sem_2;
                c_f = penalization * c_2;
            }
        }

        setSemanticClass(sem_f);
        setConfidence(c_f);

    }

    void ExtendedOctomapData::setSemanticColor(std::string semantic_class){
        if (semantic_class == "none"){
            this->semantic_r = 1.0;
            this->semantic_g = 1.0;
            this->semantic_b = 1.0;
        }
        else {
            this->semantic_r = 1.0;
            this->semantic_g = 0.0;
            this->semantic_b = 0.0;
        }
        this->semantic_a = 1.0;
    }

    void ExtendedOctomapData::setConfidenceColor(float confidence){
        this->confidence_r = confidence;
        this->confidence_g = confidence;
        this->confidence_b = confidence;
        this->confidence_a = 1.0;
    }

    std::string ExtendedOctomapData::getSemanticClass(){
        return this->semantic_class;
    }

    float ExtendedOctomapData::getConfidence(){
        return this->confidence;
    }
    

    
    ExtendedOctomapServer::ExtendedOctomapServer(const rclcpp::NodeOptions &options, const std::string node_name) : octomap_server::OctomapServer::OctomapServer(options, node_name) {
        
        RCLCPP_INFO(this->get_logger(), "Extended constructor started.");

        extended_octomap_map = std::make_shared<ExtendedOctomapMap>();

        processFreeSpace = this->declare_parameter(
            "process_free_space", processFreeSpace);
        if (processFreeSpace) {
            RCLCPP_INFO(this->get_logger(), "Computation of free voxels will not be executed.");

        }

        publishConfidence = this->declare_parameter(
            "publish_confidence", publishConfidence);
        publishSemantic = this->declare_parameter(
            "publish_semantic", publishSemantic);

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
               (*extended_octomap_map)[key] = ExtendedOctomapData();  
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

        if (insertSemanticActive){

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
            

            for (auto it = segmented_pc.begin(); it != segmented_pc.end(); ++it) {
                octomap::point3d point(it->x, it->y, it->z);                
                octomap::OcTreeKey key;

                if (m_octree->coordToKeyChecked(point, key)) {
                    if (extended_octomap_map->find(key) != extended_octomap_map->end()) {
                        (*extended_octomap_map)[key].setSemanticClass("mask");
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
    }


    void ExtendedOctomapServer::insertSemanticArrayCallback(const fruit_picking_interfaces::msg::PointcloudArray::ConstSharedPtr &segmented_pointclouds_array){

        if (insertSemanticActive){

            RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] Semantic array callback started.");
            
            if (!segmented_pointclouds_array->confidences.empty()) {

                // Save confidence vector
                std::vector<float> confidences;
                for (auto& kv : segmented_pointclouds_array->confidences) {
                    confidences.push_back(std::stof(kv.value));
                }

                // Save semantic class
                std::string semantic_class = segmented_pointclouds_array->semantic_class;

                // New unordered set to take count of the already updated octkeys
                octomap::KeySet updatedOctKeys;
                            

                RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] Entering loop...");
                // Loop thorugh all the poitclouds inside the array
                for (size_t i = 0; i < segmented_pointclouds_array->pointclouds.size(); ++i){
                    auto& segmented_pointcloud = segmented_pointclouds_array->pointclouds[i];
                    float old_confidence;
                    float confidence = confidences[i];
                    float updated_confidence;

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

                    
                    
                    bool flag = true;
                    for (auto it = segmented_pc.begin(); it != segmented_pc.end(); ++it) {
                        octomap::point3d point(it->x, it->y, it->z);                
                        octomap::OcTreeKey key;

                        if (m_octree->coordToKeyChecked(point, key)) {
                            if (updatedOctKeys.find(key) == updatedOctKeys.end()) { // If the key hasn't been updated yet
                                if (extended_octomap_map->find(key) != extended_octomap_map->end()) {
                                    if (flag){
                                        old_confidence = (*extended_octomap_map)[key].getConfidence();
                                    }
                                    (*extended_octomap_map)[key].setConfidenceMaxFusion(semantic_class, confidence);
                                    if (flag){
                                        updated_confidence = (*extended_octomap_map)[key].getConfidence();
                                        flag = false;
                                    }
                                }   
                                updatedOctKeys.insert(key); // Add the key to the set of updated keys
                            }
                        }
                    }
                    RCLCPP_ERROR(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] Object %d: Old confidence %f, new confidence %f, final confidence %f", i+1, old_confidence, confidence, updated_confidence);

                }
                
                RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] Extended octomap map updated with segmented pointcloud array.");


                if (publishConfidence){        
                    publishConfidenceMarkers(segmented_pointclouds_array->header.stamp);
                }

                if (publishSemantic){
                    publishSemanticClassMarkers(segmented_pointclouds_array->header.stamp);
                }
            }
            else{
                RCLCPP_DEBUG(this->get_logger(), "[EXTENDED OCTOMAP SERVER][insertSemanticArrayCallback] Semantic array callback finished with no effect.");
            }
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

}
RCLCPP_COMPONENTS_REGISTER_NODE(extended_octomap_server::ExtendedOctomapServer)
