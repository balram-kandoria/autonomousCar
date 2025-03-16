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
            std::string camName);

        // Methods
        void printFamily();
        void detectTag(const cv::Mat &image);

    private:
        std::string mTagFamily;
        double mTagSize;
        std::string mCamName;
        apriltag_detector_t *mtd;
        apriltag_family_t *mtf;
        
        
};