#ifndef TAGDETECTION_STRUCT_H
#define TAGDETECTION_STRUCT_H

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <variant>

// C Libraries
extern "C" {
    #include "apriltag/apriltag.h"
}

using VariantType = std::variant<int, float, std::vector<std::vector<float>>, std::vector<float>>;

struct TagDetection_struct {
    apriltag_detection_t *det; 
    int id;
    int hamming;  
    float decision_margin; 
    std::vector<std::vector<float>> cornerPoints = {
        {0,0},
        {0,0},
        {0,0},
        {0,0}
    }; // pixel locations (x, y) 4 x 2 

    std::vector<float> centerPoints;

    // Constructor
    TagDetection_struct(apriltag_detection_t *detection) 
    :   det(detection) 
         {  
            
            id = (det->id);

            hamming = (det->hamming);

            decision_margin = (det->decision_margin);

            // Populate Center Points
            for (int i=0; i<2; i++){
                centerPoints.push_back(det->c[i]);
            }

            // Populate Corner Points
            for (int i=0;i<4;i++){
                cornerPoints[i][0] = (det->p[i][0]);
                cornerPoints[i][1] = (det->p[i][1]);
            }

        }

    // Member function to display information
    void display() const;
    std::unordered_map<std::string, VariantType> returnDetection();

    
};


#endif // TAGDETECTION_H