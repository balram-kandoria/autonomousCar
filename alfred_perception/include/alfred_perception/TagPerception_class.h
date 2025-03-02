#pragma once

// Project Alfred Libraries
#include "AprilTagDetector_class.h"

// Built-in Cpp Libraries
#include <string>
#include <memory>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class TagPerception
{
    public: 
        // Constructor
        TagPerception(
            rclcpp::Node::SharedPtr perceptionNode,
            AprilTagDetector& detector_node, 
            std::string camTopic,
            std::string camName
        );

        // Methods
        void printTopic();

    private:
        // Internal Class Variables
        rclcpp::Node::SharedPtr mPerceptionNode;
        AprilTagDetector& mDetector;
        std::string mRawCamTopic;
        std::string mCamName;
        

        //Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mCamSubscription;


        // Callback
        void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        
};