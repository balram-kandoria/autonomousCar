#pragma once

// Built-in Cpp Libraries
#include <memory>
#include <string>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

class AprilTagDetector
{
    public: 
        // Constructor
        AprilTagDetector(
            std::string TagFamily, 
            double TagSize);

        // Methods
        void printFamily();
        void detectTag();

    private:
        std::string mTagFamily;
        double mTagSize;
        
};