#include <av_octomap/extended_octomap_data.hpp>

namespace extended_octomap_data {

    /**
     * @brief Constructor used to initialize the voxel extended data.
     */
    ExtendedOctomapData::ExtendedOctomapData(){}

    /**
     * @brief Constructor with hue map initialization.
     * @param hueMap Map to keep track of the coloration (hue value) of each semantic class (for visualization purposes)
     */
    ExtendedOctomapData::ExtendedOctomapData(std::map<std::string, float>& hueMap){
        setSemanticClass("none", hueMap);     
        setConfidenceNoColor(0.0);  
        setDirectConfidenceColor(1.0);  
        setInstance(0);
    }

    /**
     * @brief Sets the semantic class without setting the semantic color.
     * @param semantic_class The semantic class to set.
     */
    void ExtendedOctomapData::setSemanticClassNoColor(std::string semantic_class){
        this->semantic_class = semantic_class;
    }

    /**
     * @brief Sets the semantic class with also the semantic color.
     * @param semantic_class The semantic class to set.
     * @param hueMap Map to keep track of the coloration (hue value) of each semantic class (for visualization purposes)
     */
    void ExtendedOctomapData::setSemanticClass(std::string semantic_class, std::map<std::string, float>& hueMap){
        this->semantic_class = semantic_class;
        setSemanticColor(semantic_class, hueMap);
    }

    /**
     * @brief Sets the confidence value without setting the confidence color.
     * @param confidence The confidence value to set.
     */
    void ExtendedOctomapData::setConfidenceNoColor(float confidence){
        this->confidence = confidence;
    }

    /**
     * @brief Sets the confidence value and the confidence color with the same value of the confidence value.
     * @param confidence The confidence value to set.
     */
    void ExtendedOctomapData::setConfidence(float confidence){
        this->confidence = confidence;
        setDirectConfidenceColor(confidence);
    }

    /**
     * @brief Sets the confidence value with maximum fusion. If the semantic class is the same, 
     * it averages the confidence values. Otherwise, it keeps the semantic class with the highest confidence value. 
     * @param semantic_class The semantic class to set.
     * @param hueMap Map to keep track of the coloration (hue value) of each semantic class (for visualization purposes)
     * @param confidence The confidence value to set.
     * @param penalization The penalization factor for confidence fusion.
     */
    void ExtendedOctomapData::setConfidenceMaxFusion(std::string semantic_class, std::map<std::string, float>& hueMap, float confidence, float penalization){
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

        setSemanticClass(sem_f, hueMap);
        setConfidenceNoColor(c_f);
        setHeatConfidenceColor(c_f);
    }

    /**
     * @brief Sets the instance value without setting the instance color.
     * @param instance The instance value to set.
     */
    void ExtendedOctomapData::setInstanceNoColor(int instance){
        this->instance = instance;
    }

    /**
     * @brief Sets the instance value, as well as the instance color based on the instance value.
     * @param instance The instance value to set.
     */
    void ExtendedOctomapData::setInstance(int instance){
        this->instance = instance;
        setInstanceColor(instance);
    }

    /**
     * @brief Sets the semantic color based on the semantic class. If the semantic class is already in the hue map, 
     * it uses the existing hue value. Otherwise, it generates a new hue value.
     * @param semantic_class The semantic class to set.
     * @param hueMap Map to keep track of the coloration (hue value) of each semantic class (for visualization purposes)
     */
    void ExtendedOctomapData::setSemanticColor(std::string semantic_class, std::map<std::string, float>& hueMap){
        if (semantic_class == "none"){
            this->semantic_r = 1.0;
            this->semantic_g = 1.0;
            this->semantic_b = 1.0;
        }
        else {
            float hue;
            // Check if the semantic class already has a hue value
            auto it = hueMap.find(semantic_class);
            if (it != hueMap.end()) {
                // Use the existing hue value
                hue = it->second;
            } else {
                if (hueMap.empty()) {
                    // If the map is empty, start with a default hue, e.g., 0 for red
                    hue = 0.0;
                } else {
                    // If the map is not empty, get the hue of the last element and increment it
                    float lastHue = (--hueMap.end())->second;
                    hue = std::fmod(lastHue + 137.5, 360); // Increment and wrap around at 360 degrees
                }
                // Store the new hue in the map
                hueMap[semantic_class] = hue;
            }

            // Convert the hue to RGB for setting the object's color attributes
            float r, g, b;
            hsvToRgb(hue, 1.0, 1.0, r, g, b); // Assuming full saturation and value

            // Set the RGB values
            this->semantic_r = r;
            this->semantic_g = g;
            this->semantic_b = b;
        }
        this->semantic_a = 1.0; // Assuming alpha is always full opacity
    }

    /**
     * @brief Sets the confidence color directly.
     * @param confidence The confidence value to set.
     */
    void ExtendedOctomapData::setDirectConfidenceColor(float confidence) {
        this->confidence_r = confidence;
        this->confidence_g = confidence;
        this->confidence_b = confidence;
        this->confidence_a = 1.0; // Assuming alpha is always full opacity
    }

    /**
     * @brief Sets the confidence color manually.
     * @param r Red color value.
     * @param g Green color value.
     * @param b Blue color value.
     */
    void ExtendedOctomapData::setManualConfidenceColor(float r, float g, float b) {
        this->confidence_r = r;
        this->confidence_g = g;
        this->confidence_b = b;
        this->confidence_a = 1.0; // Assuming alpha is always full opacity
    }

    /**
     * @brief Sets the confidence color based on heat map. The heat map goes from 0 to 240 degrees.
     * 0 is red and 240 is blue. 120 is green.
     * @param confidence The confidence value to set.
     */
    void ExtendedOctomapData::setHeatConfidenceColor(float confidence) {
        
        float hue = confidence * 240.0f; // Scale confidence to [0, 240]
        float saturation = 1.0; // Full saturation
        float value = 1.0; // Full brightness/value

        // Convert HSV to RGB
        float r, g, b;
        hsvToRgb(hue, saturation, value, r, g, b);

        // Set the RGB values
        this->confidence_r = r;
        this->confidence_g = g;
        this->confidence_b = b;
        this->confidence_a = 1.0; // Assuming alpha is always full opacity
    }

    /**
     * @brief Sets the instance color. This function ensures that each instance in the OctoMap has a unique 
     * color based on its instance value, which helps in visualizing different instances distinctly. 
     * The golden angle is used to calculate the hue adjustment for each instance, ensuring an even and 
     * aesthetically pleasing distribution of colors. By leveraging the golden angle, the function can 
     * assign distinct and well-distributed colors to different instances, enhancing the clarity and 
     * effectiveness of visualizations.
     * @param instance The instance value to set.
     */
    void ExtendedOctomapData::setInstanceColor(int instance) {
        // Case when the voxel is initializated and the instance is 0
        if (instance == 0){
            this->instance_r = 1.0;
            this->instance_g = 1.0;
            this->instance_b = 1.0;
        } 
        else {
            // Define base values for hue, saturation, and value
            float baseHue = 0.0; // Starting hue
            float baseSaturation = 0.9; // Starting saturation
            float baseValue = 0.9; // Starting value

            // Calculate adjustments based on instance number
            float hueAdjustment = static_cast<float>(instance) * 137.5; // Golden angle 
            float saturationAdjustment = 0; // Optional: Adjust saturation if desired
            float valueAdjustment = 0; // Optional: Adjust value if desired

            // Apply adjustments, ensuring hue wraps around at 360
            float h = fmod(baseHue + hueAdjustment, 360);
            float s = fmin(fmax(baseSaturation + saturationAdjustment, 0), 1); // Keep s within [0, 1]
            float v = fmin(fmax(baseValue + valueAdjustment, 0), 1); // Keep v within [0, 1]

            float r, g, b;
            hsvToRgb(h, s, v, r, g, b); // Convert adjusted HSV to RGB
            this->instance_r = r;
            this->instance_g = g;
            this->instance_b = b;
        }
        this->instance_a = 1.0; // Assuming alpha is always 1.0
    }

    /**
     * @brief Sets the number of points that are inside the voxel referring to this OctoMap data.
     * @param count The points count to set.
     */
    void ExtendedOctomapData::setPointsCount(int count){
        this->semantic_class_points_count = count;
    }

    /**
     * @brief Gets the semantic class.
     * @return The semantic class.
     */
    std::string ExtendedOctomapData::getSemanticClass(){
        return this->semantic_class;
    }

    /**
     * @brief Gets the confidence value.
     * @return The confidence value.
     */
    float ExtendedOctomapData::getConfidence(){
        return this->confidence;
    }

    /**
     * @brief Gets the instance value.
     * @return The instance value.
     */
    int ExtendedOctomapData::getInstance(){
        return this->instance;
    }

    /**
     * @brief Gets the points count.
     * @return The points count.
     */
    int ExtendedOctomapData::getPointsCount(){
        return this->semantic_class_points_count;
    }

    /**
     * @brief Converts HSV color values to RGB color values.
     * @param h Hue value.
     * @param s Saturation value.
     * @param v Value (brightness).
     * @param r Reference to store the red color value.
     * @param g Reference to store the green color value.
     * @param b Reference to store the blue color value.
     */
    void hsvToRgb(float h, float s, float v, float &r, float &g, float &b) {
        int i;
        float f, p, q, t;
        if (s == 0) {
            r = g = b = v;
            return;
        }
        h /= 60; // sector 0 to 5
        i = floor(h);
        f = h - i; // factorial part of h
        p = v * (1 - s);
        q = v * (1 - s * f);
        t = v * (1 - s * (1 - f));
        switch (i) {
            case 0:
                r = v;
                g = t;
                b = p;
                break;
            case 1:
                r = q;
                g = v;
                b = p;
                break;
            case 2:
                r = p;
                g = v;
                b = t;
                break;
            case 3:
                r = p;
                g = q;
                b = v;
                break;
            case 4:
                r = t;
                g = p;
                b = v;
                break;
            default: // case 5:
                r = v;
                g = p;
                b = q;
                break;
        }
    }

    

}