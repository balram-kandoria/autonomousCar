#include "AprilTagDetector_class.h"

// Built-in Cpp Libraries
#include <memory>
#include <string>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

AprilTagDetector::AprilTagDetector(
    std::string TagFamily, 
    double TagSize) 
{
    mTagFamily = TagFamily;
    mTagSize = TagSize;
}

void AprilTagDetector::printFamily() 
{
    std::cout << "April Tag Family: " << mTagFamily << "\n";
}

void AprilTagDetector::detectTag()
{
    std::cout << "Running Tag Detector" << "\n";
}