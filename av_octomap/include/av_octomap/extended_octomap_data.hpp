// The ExtendedOctomapData class is designed to manage and manipulate semantic infor-
// mation. The objects of this class are the values of the hash map related to the semantic
// OctoMap.

#pragma once
#ifndef _EXTENDED_OCTOMAP_DATA_HPP_
#define _EXTENDED_OCTOMAP_DATA_HPP_

#include <string>
#include <map>
#include <cmath>

namespace extended_octomap_data{

    /**
    * @brief Converts HSV color values to RGB color values.
    * @param h Hue value.
    * @param s Saturation value.
    * @param v Value (brightness).
    * @param r Reference to store the red color value.
    * @param g Reference to store the green color value.
    * @param b Reference to store the blue color value.
    */
    void hsvToRgb(float h, float s, float v, float &r, float &g, float &b);

    class ExtendedOctomapData{
    
    public:

        /**
        * @brief Constructor used to initialize the voxel extended data.
        */
        ExtendedOctomapData();

        /**
        * @brief Constructor with hue map initialization.
        * @param hueMap Map to keep track of the coloration (hue value) of each semantic class (for visualization purposes)
        */
        ExtendedOctomapData(std::map<std::string, float>& hueMap);

        /**
        * @brief Sets the semantic class without setting the semantic color.
        * @param semantic_class The semantic class to set.
        */
        void setSemanticClassNoColor(std::string semantic_class);

        /**
        * @brief Sets the semantic class with also the semantic color.
        * @param semantic_class The semantic class to set.
        * @param hueMap Map to keep track of the coloration (hue value) of each semantic class (for visualization purposes)
        */
        void setSemanticClass(std::string semantic_class, std::map<std::string, float>& hueMap);

        /**
        * @brief Sets the confidence value without setting the confidence color.
        * @param confidence The confidence value to set.
        */
        void setConfidenceNoColor(float confidence);

        /**
        * @brief Sets the confidence value and the confidence color with the same value of the confidence value.
        * @param confidence The confidence value to set.
        */
        void setConfidence(float confidence);

        /**
        * @brief Sets the confidence value with maximum fusion. If the semantic class is the same, 
        * it averages the confidence values. Otherwise, it keeps the semantic class with the highest confidence value. 
        * @param semantic_class The semantic class to set.
        * @param hueMap Map to keep track of the coloration (hue value) of each semantic class (for visualization purposes)
        * @param confidence The confidence value to set.
        * @param penalization The penalization factor for confidence fusion.
        */
        void setConfidenceMaxFusion(std::string semantic_class, std::map<std::string, float>& hueMap, float confidence, float penalization = 0.9);

        /**
        * @brief Sets the instance value without setting the instance color.
        * @param instance The instance value to set.
        */
        void setInstanceNoColor(int instance);

        /**
        * @brief Sets the instance value, as well as the instance color based on the instance value.
        * @param instance The instance value to set.
        */
        void setInstance(int instance);

        /**
        * @brief Sets the semantic color based on the semantic class. If the semantic class is already in the hue map, 
        * it uses the existing hue value. Otherwise, it generates a new hue value.
        * @param semantic_class The semantic class to set.
        * @param hueMap Map to keep track of the coloration (hue value) of each semantic class (for visualization purposes)
        */
        void setSemanticColor(std::string semantic_class, std::map<std::string, float>& hueMap);

        /**
        * @brief Sets the confidence color directly.
        * @param confidence The confidence value to set.
        */
        void setDirectConfidenceColor(float confidence);

        /**
        * @brief Sets the confidence color manually.
        * @param r Red color value.
        * @param g Green color value.
        * @param b Blue color value.
        */
        void setManualConfidenceColor(float r, float g, float b);

        /**
        * @brief Sets the confidence color based on heat map. The heat map goes from 0 to 240 degrees.
        * 0 is red and 240 is blue. 120 is green.
        * @param confidence The confidence value to set.
        */
        void setHeatConfidenceColor(float confidence);

        /**
        * @brief Sets the instance color. This function ensures that each instance in the OctoMap has a unique 
        * color based on its instance value, which helps in visualizing different instances distinctly. 
        * The golden angle is used to calculate the hue adjustment for each instance, ensuring an even and 
        * aesthetically pleasing distribution of colors. By leveraging the golden angle, the function can 
        * assign distinct and well-distributed colors to different instances, enhancing the clarity and 
        * effectiveness of visualizations.
        * @param instance The instance value to set.
        */
        void setInstanceColor(int instance);

        /**
        * @brief Sets the number of points that are inside the voxel referring to this OctoMap data.
        * @param count The points count to set.
        */
        void setPointsCount(int count);

        /**
        * @brief Gets the semantic class.
        * @return The semantic class.
        */
        std::string getSemanticClass();

        /**
        * @brief Gets the confidence value.
        * @return The confidence value.
        */
        float getConfidence();

        /**
        * @brief Gets the instance value.
        * @return The instance value.
        */
        int getInstance();

        /**
        * @brief Gets the points count.
        * @return The points count.
        */
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