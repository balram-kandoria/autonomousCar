// Project Alfred Libraries
#include "AprilTagDetector_class.h"
#include "TagPerception_class.h"

// Built-in Cpp Libraries
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.h"

// Other Libraries
#include "apriltag/apriltag.h"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include "yaml-cpp/yaml.h"

// Structures
#include "TagDetection_struct.h"

// C Libraries
extern "C" {
    #include "apriltag/apriltag.h"
    #include "apriltag/apriltag_pose.h"
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

TagPerception::TagPerception(
    rclcpp::Node::SharedPtr perceptionNode,
    AprilTagDetector& detector, 
    std::string camTopic,
    std::string camName) 
    : 
    _PerceptionNode(perceptionNode),
    _Detector(detector), 
    _RawCamTopic(std::move(camTopic)),
    _CamName(std::move(camName))
    
{
    _CamSubscription = _PerceptionNode->create_subscription<sensor_msgs::msg::Image>(
        _RawCamTopic, 10,
        std::bind(&TagPerception::imageCallback, this, std::placeholders::_1)
    );

    std::cout << "Initializing Perception for [" << _CamName << "]\n";

}

void TagPerception::printTopic() 
{
    std::cout << "April Tag Topic: " << _RawCamTopic << "\n";
}

void TagPerception::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{

    cv::Mat image = cv_bridge::toCvCopy(msg,"bgr8")->image;

    std::vector<TagDetection_struct> detection_list = _Detector.detectTag(image);

    // detection.display();

    std::string key = "id";

    if (detection_list.size() > 0) {
        std::cout << detection_list[0].x << "\n";
    };
    
}
