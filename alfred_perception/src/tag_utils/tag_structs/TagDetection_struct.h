#ifndef TAGDETECTION_STRUCT_H
#define TAGDETECTION_STRUCT_H

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <variant>

// Other Libraries
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

// C Libraries
extern "C" {
    #include "apriltag/apriltag.h"
}

struct TagDetection_struct {
    apriltag_detection_t *det; 
    int id = -1;
    int hamming = 0;  
    float decision_margin = 0.0; 
    std::vector<cv::Point2d> cornerPoints = {
        {0,0},
        {0,0},
        {0,0},
        {0,0}
    }; // pixel locations (x, y) 4 x 2 

    std::vector<float> centerPoints = {0,0};

    double x = 0.0; 
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    double _TagSize = 0.0;


    // Constructor
    TagDetection_struct(apriltag_detection_t *detection = {}, double TagSize = 0) 
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
                cornerPoints[i] = {det->p[i][0], det->p[i][1]};
                // cornerPoints[i][1] = ();
            }

            _TagSize = TagSize;

        }

    // Member function to display information
    void display() const;
    cv::Mat drawDetection(cv::Mat img);

};


#endif // TAGDETECTION_H