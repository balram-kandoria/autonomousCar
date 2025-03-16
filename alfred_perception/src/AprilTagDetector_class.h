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

using VariantType = std::variant<int, float, std::vector<std::vector<float>>, std::vector<float>>;

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
        std::unordered_map<std::string, VariantType> detectTag(const cv::Mat &image);

    private:
        std::string _TagFamily;
        double _TagSize;
        std::string _CamName;
        apriltag_detector_t *_td;
        apriltag_family_t *_tf;
        
        
};