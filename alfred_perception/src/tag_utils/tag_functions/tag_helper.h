// my_functions.h
#ifndef MY_FUNCTIONS_H
#define MY_FUNCTIONS_H

// Other Libraries
#include "apriltag/apriltag.h"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include "yaml-cpp/yaml.h"

// Structures
#include "TagDetection_struct.h"

int add(int a, int b);
double multiply(double x, double y);
cv::Mat add_Detection_to_Image(apriltag_detection_t *det, cv::Mat frame);

#endif