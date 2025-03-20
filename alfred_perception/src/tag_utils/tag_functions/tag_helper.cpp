// Other Libraries
#include "apriltag/apriltag.h"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include "yaml-cpp/yaml.h"

// Structures
#include "TagDetection_struct.h"

// my_functions.cpp
#include "tag_helper.h"

int add(int a, int b) {
    return a + b;
}

double multiply(double x, double y) {
    return x * y;
}

cv::Mat add_Detection_to_Image(apriltag_detection_t *det, cv::Mat frame) {

    cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]),
        cv::Point(det->p[1][0], det->p[1][1]),
        cv::Scalar(0, 0xff, 0), 2);
    cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]),
        cv::Point(det->p[3][0], det->p[3][1]),
        cv::Scalar(0, 0, 0xff), 2);
    cv::line(frame, cv::Point(det->p[1][0], det->p[1][1]),
        cv::Point(det->p[2][0], det->p[2][1]),
        cv::Scalar(0xff, 0, 0), 2);
    cv::line(frame, cv::Point(det->p[2][0], det->p[2][1]),
        cv::Point(det->p[3][0], det->p[3][1]),
        cv::Scalar(0xff, 0, 0), 2);

    std::stringstream ss;
    ss << det->id;
    cv::String text = ss.str();
    int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontscale = 1.0;
    int baseline;
    
    cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
                                    &baseline);
    cv::putText(frame, text, cv::Point(det->c[0]-textsize.width/2,
                               det->c[1]+textsize.height/2),
            fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);

    return frame;
}