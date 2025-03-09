#ifndef TAGDETECTION_STRUCT_H
#define TAGDETECTION_STRUCT_H

#include <iostream>
#include <string>
#include <vector>

// C Libraries
extern "C" {
    #include "apriltag/apriltag.h"
}

struct TagDetection_struct {
    apriltag_detection_t* det; 
    int id;
    int hamming;  
    float decision_margin; 
    std::vector<std::vector<float>> cornerPoints; // pixel locations (x, y) 4 x 2 
    std::vector<float> centerPoints;

    // Constructor
    TagDetection_struct(apriltag_detection_t detection) 
    :   det(&detection) 
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
                cornerPoints[i].push_back(det->p[i][0]);
                cornerPoints[i].push_back(det->p[i][1]);
            }
        }
        // cornerPoints(corner), centerPoints(center) 

    // Member function to display information
    void display() const;
};

#endif // TAGDETECTION_H