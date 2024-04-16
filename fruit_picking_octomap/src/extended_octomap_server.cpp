#include <fruit_picking_octomap/extended_octomap_server.hpp>

namespace extended_octomap_server {
    
    ExtendedOctomapServer::ExtendedOctomapServer(const rclcpp::NodeOptions &options, const std::string node_name) : octomap_server::OctomapServer::OctomapServer(options, node_name) {
        
        RCLCPP_INFO(this->get_logger(), "Extended constructor started.");

        extended_octomap_map = std::make_shared<ExtendedOctomapMap>();

        m_processFreeSpace = this->declare_parameter(
            "process_free_space", m_processFreeSpace);
        if (m_processFreeSpace) {
            RCLCPP_INFO(this->get_logger(), "Computation of free voxels will not be executed.");

        }
        publishConfidence = this->declare_parameter(
            "publish_confidence", publishConfidence);
        publishSemantic = this->declare_parameter(
            "publish_semantic", publishSemantic);

        this->onInit();
    }

    ExtendedOctomapServer::~ExtendedOctomapServer(){}



    void ExtendedOctomapServer::onInit() {
        
        RCLCPP_INFO(this->get_logger(), "On init of extended octomap server started.");

        this->m_reducedPointCloudSub = std::make_shared<
            message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
                this, "reduced_cloud_in", rmw_qos_profile_sensor_data);
        
        this->m_tfReducedPointCloudSub = std::make_shared<tf2_ros::MessageFilter<
            sensor_msgs::msg::PointCloud2>>(
                *buffer_, m_worldFrameId, 5,
                this->get_node_logging_interface(),
                this->get_node_clock_interface(),
                std::chrono::seconds(1));
        this->m_tfReducedPointCloudSub->connectInput(*m_reducedPointCloudSub);
        this->m_tfReducedPointCloudSub->registerCallback(
            std::bind(&ExtendedOctomapServer::insertSemanticCallback, this, ph::_1));


        RCLCPP_INFO(this->get_logger(), "Subscription to semantic topics done.");

        rclcpp::QoS qos(rclcpp::KeepLast(3));
        if (publishConfidence){
            this->confidenceMarkerPub = this->create_publisher<
                visualization_msgs::msg::MarkerArray>(
                    "confidence_vis_array", qos);
        }
        if (publishSemantic){
            this->semanticClassMarkerPub = this->create_publisher<
                visualization_msgs::msg::MarkerArray>(
                    "semantic_class_vis_array", qos);
        }
    }



    void ExtendedOctomapServer::insertCloudCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud){
        
        OctomapServer::insertCloudCallback(cloud);

        if (publishConfidence){        
            publishConfidenceMarkers(cloud->header.stamp);
        }

        if (publishSemantic){
            publishSemanticClassMarkers(cloud->header.stamp);
        }
        
    }


    void ExtendedOctomapServer::insertScan(
        const geometry_msgs::msg::Vector3 &sensorOriginTf,
        const PCLPointCloud& ground,
        const PCLPointCloud& nonground) {
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
            if (m_processFreeSpace){
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
                if (m_processFreeSpace){
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
                if (m_processFreeSpace){
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
        if (m_processFreeSpace){
            RCLCPP_INFO(this->get_logger(), "Time lapse [insertion of the pointcloud in free and occupied cells] %f", elapsed_seconds.count());
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Time lapse [insertion of the pointcloud occupied cells] %f", elapsed_seconds.count());
        }


        
        if (m_processFreeSpace){
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

        // populate the global sets
        global_occupied_cells.insert(occupied_cells.begin(), occupied_cells.end());
        if (m_processFreeSpace){
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



    void ExtendedOctomapServer::insertSemanticCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &reduced_cloud){

        RCLCPP_INFO(this->get_logger(), "Semantic callback started.");

        // From pointcloud message to Pointcloud data structure
        PCLPointCloud reduced_pc;
        pcl::fromROSMsg(*reduced_cloud, reduced_pc);

        // Conversion of the pointcloud from sensor frame to world frame
        Eigen::Matrix4f sensorToWorld;
        geometry_msgs::msg::TransformStamped sensorToWorldTf;
        try {
            if (!this->buffer_->canTransform(
                    m_worldFrameId, reduced_cloud->header.frame_id,
                    reduced_cloud->header.stamp)) {
                throw "Failed";
            }
            
            sensorToWorldTf = this->buffer_->lookupTransform(
                m_worldFrameId, reduced_cloud->header.frame_id,
                reduced_cloud->header.stamp);
            sensorToWorld = pcl_ros::transformAsMatrix(sensorToWorldTf);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s",ex.what());
            return;
        }
        pcl::transformPointCloud(reduced_pc, reduced_pc, sensorToWorld);
        

        for (auto it = reduced_pc.begin(); it != reduced_pc.end(); ++it) {
            octomap::point3d point(it->x, it->y, it->z);                
            octomap::OcTreeKey key;

            if (m_octree->coordToKeyChecked(point, key)) {
                (*extended_octomap_map)[key] = ExtendedOctomapData(tomato);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Extended octomap map updated.");

    }



    void ExtendedOctomapServer::publishConfidenceMarkers(const rclcpp::Time &rostime) const {
        
        // RCLCPP_INFO(this->get_logger(), "Publishing markers of confidence...");

        bool publishConfidenceMarkers = confidenceMarkerPub->get_subscription_count() > 0;


    }




    void ExtendedOctomapServer::publishSemanticClassMarkers(const rclcpp::Time &rostime) const {
        
        // RCLCPP_INFO(this->get_logger(), "Publishing markers of classes...");

        bool publishSemanticMarkers = semanticClassMarkerPub->get_subscription_count() > 0;
        size_t octomap_size = m_octree->size();

        // RCLCPP_INFO(this->get_logger(), "Tree size: %lu", octomap_size);


        if (octomap_size <= 1) {
            RCLCPP_WARN(
                this->get_logger(),
                "Nothing to publish, octree is empty");
            return;
        }

        // init markers for free space:
        visualization_msgs::msg::MarkerArray semanticVis;
        // each array stores all cubes of a different size, one for each depth level:
        semanticVis.markers.resize(m_treeDepth+1);

        // RCLCPP_INFO(this->get_logger(), "Tree depth: %u", m_treeDepth);
        // RCLCPP_INFO(this->get_logger(), "Array length: %lu", confidenceVis.markers.size());



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
                _color.r = it->second.r;
                _color.g = it->second.g;
                _color.b = it->second.b;
                _color.a = it->second.a;                            
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

        RCLCPP_INFO(this->get_logger(), "Markers of classes published.");

    }    

}
RCLCPP_COMPONENTS_REGISTER_NODE(extended_octomap_server::ExtendedOctomapServer)
