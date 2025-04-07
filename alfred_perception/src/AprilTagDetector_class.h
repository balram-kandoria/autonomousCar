#pragma once

// Built-in Cpp Libraries
#include <memory>
#include <string>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// Other Libraries
#include "apriltag/apriltag.h"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

// My changes

// Structures
#include "TagDetection_struct.h"

// C Libraries
extern "C" {
    #include "apriltag/apriltag.h"
    #include "apriltag/tag36h11.h"
    #include "apriltag/tag25h9.h"
    #include "apriltag/tag16h5.h"
    #include "apriltag/tagCircle21h7.h"
    #include "apriltag/tagCircle49h12.h"
    #include "apriltag/tagCustom48h12.h"
    #include "apriltag/tagStandard41h12.h"
    #include "apriltag/tagStandard52h13.h"
    #include "apriltag/common/getopt.h"
    }

class AprilTagDetector
{
    public: 
        // Constructor
        AprilTagDetector(
            std::string TagFamily, 
            double TagSize,
            std::string camName,
            cv::Mat camDistortionCoeff,
            cv::Mat camMatrix,
            bool visualize,
            bool debug);

        // Methods
        void printFamily();
        std::vector<TagDetection_struct> detectTag(const cv::Mat &image);

    private:
        std::string _TagFamily;
        double _TagSize;
        std::string _CamName;
        cv::Mat _camDistortionCoeff;
        cv::Mat _camMatrix;
        cv::Mat _objPt;
        bool _visualize;
        bool _debug;
        apriltag_detector_t *_td;
        apriltag_family_t *_tf;
        
        
};