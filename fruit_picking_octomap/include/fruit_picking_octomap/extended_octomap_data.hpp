#pragma once
#ifndef _EXTENDED_OCTOMAP_DATA_HPP_
#define _EXTENDED_OCTOMAP_DATA_HPP_

#include <string>
#include <map>
#include <cmath>

namespace extended_octomap_data{

    // HSV to RGB conversion function
    void hsvToRgb(float h, float s, float v, float &r, float &g, float &b);

    class ExtendedOctomapData{
    
    public:

        // Constructor used to initialize the voxel extended data. Called by the insertCloud callback
        ExtendedOctomapData();

        ExtendedOctomapData(std::map<std::string, float>& hueMap);

        void setSemanticClassNoColor(std::string semantic_class);

        void setSemanticClass(std::string semantic_class, std::map<std::string, float>& hueMap);

        void setConfidenceNoColor(float confidence);

        void setConfidence(float confidence);

        void setConfidenceMaxFusion(std::string semantic_class, std::map<std::string, float>& hueMap, float confidence, float penalization = 0.9);

        void setInstanceNoColor(int instance);

        void setInstance(int instance);

        void setSemanticColor(std::string semantic_class, std::map<std::string, float>& hueMap);

        void setDirectConfidenceColor(float confidence);

        void setHeatConfidenceColor(float confidence);

        void setInstanceColor(int instance);

        void setPointsCount(int count);

        std::string getSemanticClass();

        float getConfidence();

        int getInstance();

        int getPointsCount();

        float semantic_r, semantic_g, semantic_b, semantic_a;
        float confidence_r, confidence_g, confidence_b, confidence_a;
        float instance_r, instance_g, instance_b, instance_a;


    protected:

        float confidence;
        std::string semantic_class;
        int instance;
        int semantic_class_points_count;
    };

}

#endif