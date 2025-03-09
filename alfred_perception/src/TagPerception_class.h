#pragma once

// Project Alfred Libraries
#include "AprilTagDetector_class.h"
#include "TagPerception_class.h"

// Built-in Cpp Libraries
#include <iostream>
#include <iomanip>
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
        apriltag_detector_t *mtd;
        apriltag_family_t *mtf;
        

        //Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mCamSubscription;


        // Callback
        void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        
};