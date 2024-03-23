#include <fruit_picking_octomap/extended_octomap_server.hpp>

namespace extended_octomap_server {
    
    ExtendedOctomapServer::ExtendedOctomapServer(const rclcpp::NodeOptions &options, const std::string node_name) : octomap_server::OctomapServer::OctomapServer(options, node_name) {
        
        RCLCPP_INFO(this->get_logger(), "Extended constructor started.");

        extended_octomap_map = std::make_shared<ExtendedOctomapMap>();

        this->onInit();
    }

    ExtendedOctomapServer::~ExtendedOctomapServer(){}



    void ExtendedOctomapServer::onInit() {
        
        // subcribe to semantic topics

        RCLCPP_INFO(this->get_logger(), "On init of extended octomap server started.");


        rclcpp::QoS qos(rclcpp::KeepLast(3));
        this->confidenceMarkerPub = this->create_publisher<
            visualization_msgs::msg::MarkerArray>(
                "confidence_vis_array", qos);
        this->semanticClassMarkerPub = this->create_publisher<
            visualization_msgs::msg::MarkerArray>(
                "semantic_class_vis_array", qos);
    }



    void ExtendedOctomapServer::insertCloudCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud){
        auto start = std::chrono::steady_clock::now();
        
        //
        // ground filtering in base frame
        //
        PCLPointCloud pc; // input cloud for filtering and ground-detection
        pcl::fromROSMsg(*cloud, pc);
        
        Eigen::Matrix4f sensorToWorld; // matrix of size 4 composed of float
        geometry_msgs::msg::TransformStamped sensorToWorldTf;
        try {
            if (!this->buffer_->canTransform(
                    m_worldFrameId, cloud->header.frame_id,
                    cloud->header.stamp)) {
                throw "Failed";
            }
            
            // RCLCPP_INFO(this->get_logger(), "Can transform");

            sensorToWorldTf = this->buffer_->lookupTransform(
                m_worldFrameId, cloud->header.frame_id,
                cloud->header.stamp);
            sensorToWorld = pcl_ros::transformAsMatrix(sensorToWorldTf);
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
                RCLCPP_ERROR(this->get_logger(), "%s %", msg, ex.what());
                return;
            }

            Eigen::Matrix4f sensorToBase =
                pcl_ros::transformAsMatrix(sensorToBaseTf);
            Eigen::Matrix4f baseToWorld = 
                pcl_ros::transformAsMatrix(baseToWorldTf);

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
        RCLCPP_INFO(this->get_logger(), "Time lapse [from receiving pt to final octomap insertion] %f", elapsed_seconds.count());

        insertSemantic();
        
        publishAll(cloud->header.stamp);
        publishConfidenceMarkers(cloud->header.stamp);
        publishSemanticClassMarkers(cloud->header.stamp);

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
        // insert ground points only as free:
        for (auto it = ground.begin(); it != ground.end(); ++it) {
            octomap::point3d point(it->x, it->y, it->z);
            // maxrange check
            if ((m_maxRange > 0.0) && ((point - sensorOrigin).norm() > m_maxRange) ) {
                point = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
            }

            // only clear space (ground points)
            if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)) {
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
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
                if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)) {
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());
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

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        RCLCPP_INFO(this->get_logger(), "Time lapse [insertion of the pointcloud in free and occupied cells] %f", elapsed_seconds.count());
        
        // mark free cells only if not seen occupied in this cloud
        for(auto it = free_cells.begin(), end=free_cells.end();
            it!= end; ++it){
            if (occupied_cells.find(*it) == occupied_cells.end()){ // if the free cell is not found in occupied cell, and so if this cell is a real free cell
                m_octree->updateNode(*it, false);
            }
        }

        // now mark all occupied cells:
        for (auto it = occupied_cells.begin(),
                 end=occupied_cells.end(); it!= end; it++) {
            m_octree->updateNode(*it, true);
        }

        // TODO: eval lazy+updateInner vs. proper insertion
        // non-lazy by default (updateInnerOccupancy() too slow for large maps)
        //m_octree->updateInnerOccupancy();
        octomap::point3d minPt, maxPt;
        /* todo
        ROS_DEBUG_STREAM("Bounding box keys (before): "
                         << m_updateBBXMin[0] << " "
                         << m_updateBBXMin[1] << " "
                         << m_updateBBXMin[2] << " / "
                         <<m_updateBBXMax[0] << " "
                         << m_updateBBXMax[1] << " "
                         << m_updateBBXMax[2]);
        */
        
        // TODO: we could also limit the bbx to be within the map bounds here
        // (see publishing check)
        
        minPt = m_octree->keyToCoord(m_updateBBXMin);
        maxPt = m_octree->keyToCoord(m_updateBBXMax);
        /* todo
        ROS_DEBUG_STREAM("Updated area bounding box: "<< minPt << " - "<<maxPt);
        ROS_DEBUG_STREAM("Bounding box keys (after): "
                         << m_updateBBXMin[0] << " " << m_updateBBXMin[1]
                         << " " << m_updateBBXMin[2] << " / "
                         << m_updateBBXMax[0] << " "<< m_updateBBXMax[1]
                         << " "<< m_updateBBXMax[2]);
        */

        if (m_compressMap) {
            m_octree->prune();
        }
        
#ifdef COLOR_OCTOMAP_SERVER
        if (colors) {
            delete[] colors;
            colors = NULL;
        }
#endif
    }



    void ExtendedOctomapServer::insertSemantic(){

        RCLCPP_INFO(this->get_logger(), "Semantic function started.");

        ExtendedOctomapData test_extended_octomap_data(0.7, "tomato");

        // Insert into the ExtendedOctomapMap keys the occupied OctKeys
        for(const auto& key : occupied_cells) {
            (*extended_octomap_map)[key] = test_extended_octomap_data;           
        }

        RCLCPP_INFO(this->get_logger(), "Extended octomap map updated.");

    }



    void ExtendedOctomapServer::publishConfidenceMarkers(const rclcpp::Time &rostime) const {
        
        RCLCPP_INFO(this->get_logger(), "Publishing markers of confidences...");

        bool publishConfidenceMarkers = confidenceMarkerPub->get_subscription_count() > 0;
        size_t octomap_size = m_octree->size();

        RCLCPP_INFO(this->get_logger(), "Tree size: %lu", octomap_size);


        if (octomap_size <= 1) {
            RCLCPP_WARN(
                this->get_logger(),
                "Nothing to publish, octree is empty");
            return;
        }

        // init markers for free space:
        visualization_msgs::msg::MarkerArray confidenceVis;
        // each array stores all cubes of a different size, one for each depth level:
        confidenceVis.markers.resize(m_treeDepth+1);

        RCLCPP_INFO(this->get_logger(), "Tree depth: %u", m_treeDepth);
        RCLCPP_INFO(this->get_logger(), "Array length: %lu", confidenceVis.markers.size());



        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, 0.0);        
        geometry_msgs::msg::Pose pose;
        pose.orientation = tf2::toMsg(quaternion);

        RCLCPP_INFO(this->get_logger(), "Max tree depth: %d", m_maxTreeDepth);


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
                _color.r = it->second.confidence;
                _color.g = it->second.confidence;
                _color.b = it->second.confidence;
                _color.a = 1.0;                            
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

        RCLCPP_INFO(this->get_logger(), "Markers of confidences published.");

    }



    void ExtendedOctomapServer::publishSemanticClassMarkers(const rclcpp::Time &rostime) const {
        
        RCLCPP_INFO(this->get_logger(), "Publishing markers of classes...");


        bool publishSemanticClassMarkers = semanticClassMarkerPub->get_subscription_count() > 0;



    }


    

}
RCLCPP_COMPONENTS_REGISTER_NODE(extended_octomap_server::ExtendedOctomapServer)
