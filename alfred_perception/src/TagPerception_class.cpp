#include "AprilTagDetector_class.h"
#include "TagPerception_class.h"

#include <iostream>
#include <string>
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"


TagPerception::TagPerception(
    rclcpp::Node::SharedPtr perceptionNode,
    AprilTagDetector& detector, 
    std::string camTopic,
    std::string camName) 
    : 
    mPerceptionNode(perceptionNode),
    mDetector(detector), 
    mRawCamTopic(std::move(camTopic)),
    mCamName(std::move(camName))
    
{
    mCamSubscription = mPerceptionNode->create_subscription<sensor_msgs::msg::Image>(
        mRawCamTopic, 10,
        std::bind(&TagPerception::imageCallback, this, std::placeholders::_1)
    );
}

void TagPerception::printTopic() 
{
    std::cout << "April Tag Topic: " << mRawCamTopic << "\n";
}

void TagPerception::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::cout << "Received an image frame!\n" << std::endl;
    std::cout << "Message: " << msg << std::endl;

    cv::Mat image = cv_bridge::toCvCopy(msg,"bgr8")->image;

    cv::imshow(mCamName, image);  
    int k = cv::waitKey(1); // Wait for a keystroke in the window
    // cv::destroyWindow(mCamName);
}