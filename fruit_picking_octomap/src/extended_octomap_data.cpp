#include <fruit_picking_octomap/extended_octomap_data.hpp>

namespace extended_octomap_data {

    ExtendedOctomapData::ExtendedOctomapData(){
        setSemanticClass("none");     
        setConfidenceNoColor(0.0);  
        setDirectConfidenceColor(1.0);  
        setInstance(0);
    }

    void ExtendedOctomapData::setSemanticClassNoColor(std::string semantic_class){
        this->semantic_class = semantic_class;
    }

    void ExtendedOctomapData::setSemanticClass(std::string semantic_class){
        this->semantic_class = semantic_class;
        setSemanticColor(semantic_class);
    }

    void ExtendedOctomapData::setConfidenceNoColor(float confidence){
        this->confidence = confidence;
    }

    void ExtendedOctomapData::setConfidence(float confidence){
        this->confidence = confidence;
        setDirectConfidenceColor(confidence);
    }

    void ExtendedOctomapData::setConfidenceMaxFusion(std::string semantic_class, float confidence, float penalization){
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
        setConfidenceNoColor(c_f);
        setHeatConfidenceColor(c_f);
    }

    void ExtendedOctomapData::setInstanceNoColor(int instance){
        this->instance = instance;
    }

    void ExtendedOctomapData::setInstance(int instance){
        this->instance = instance;
        setInstanceColor(instance);
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

    void ExtendedOctomapData::setDirectConfidenceColor(float confidence) {
        this->confidence_r = confidence;
        this->confidence_g = confidence;
        this->confidence_b = confidence;
        this->confidence_a = 1.0; // Assuming alpha is always full opacity
    }

    void ExtendedOctomapData::setHeatConfidenceColor(float confidence) {
        // Map confidence to hue (0 to 120 degrees, where 0 is red and 120 is green)
        float hue = confidence * 240.0f; // 0.0 -> 0° (red), 1.0 -> 120° (green)
        float saturation = 1.0; // Full saturation
        float value = 1.0; // Full brightness/value

        // Convert HSV to RGB using the provided lambda function
        auto hsvToRgb = [](float h, float s, float v, float &r, float &g, float &b) {
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
        };

        // Use the lambda to convert HSV to RGB
        float r, g, b;
        hsvToRgb(hue, saturation, value, r, g, b);

        // Set the RGB values
        this->confidence_r = r;
        this->confidence_g = g;
        this->confidence_b = b;
        this->confidence_a = 1.0; // Assuming alpha is always full opacity
    }

    void ExtendedOctomapData::setInstanceColor(int instance) {
        // Inline function to convert HSV to RGB
        auto hsvToRgb = [](float h, float s, float v, float &r, float &g, float &b) {
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
        };
        // case when the voxel is initializated and the instance is 0
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

    std::string ExtendedOctomapData::getSemanticClass(){
        return this->semantic_class;
    }

    float ExtendedOctomapData::getConfidence(){
        return this->confidence;
    }

    int ExtendedOctomapData::getInstance(){
        return this->instance;
    }

}