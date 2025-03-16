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

using VariantType = std::variant<int, float, std::vector<std::vector<float>>, std::vector<float>>;

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

    std::cout << "Initializing Perception for [" << mCamName << "]\n";

   
}

void TagPerception::printTopic() 
{
    std::cout << "April Tag Topic: " << mRawCamTopic << "\n";
}

void TagPerception::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{

    cv::Mat image = cv_bridge::toCvCopy(msg,"bgr8")->image;

    std::cout << "One\n";
    std::unordered_map<std::string, VariantType> detection = mDetector.detectTag(image);

    std::cout << "Two\n";
    // detection.display();
    std::cout << "Three\n";

    // cv::imshow(mCamName, image);  
    // int k = cv::waitKey(1); // Wait for a keystroke in the window
    // cv::destroyWindow(mCamName);

    // Create Detector
    // apriltag_family_t *tf = NULL;
    // tf = tag36h11_create();

    // apriltag_detector_t *mtd = apriltag_detector_create();
    // apriltag_detector_add_family(mtd, tf);

    

        
        
        
        
        


             

}
