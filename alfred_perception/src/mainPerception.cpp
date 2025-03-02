// Project Alfred Libraries
#include "AprilTagDetector_class.h"
#include "TagPerception_class.h"

// Built-in Cpp Libraries
#include <string>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"



int main(int argc, char * argv[])
{
    std::string rawBackCamTopic = "/vehicle/sensor/back_cam/camera_image";
    std::string rawFrontCamTopic = "/vehicle/sensor/front_cam/camera_image";

    std::string tagFamily = "Tag36h11";

    // Initialize Node
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr perception_node = rclcpp::Node::make_shared("perception_node");

    
    AprilTagDetector Tag1{
        tagFamily, 
        0.150};

    AprilTagDetector Tag2{
        tagFamily, 
        0.150};

    // Tag.printFamily();

    TagPerception Cam2{
        perception_node,
        Tag2,
        rawFrontCamTopic,
        "Front Cam"
    };

    TagPerception Cam1{
        perception_node,
        Tag1,
        rawBackCamTopic,
        "Back Cam"
    };

    

    Cam1.printTopic();

    rclcpp::spin(perception_node);
    rclcpp::shutdown();
    return 0;
}