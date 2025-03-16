// Project Alfred Libraries
#include "AprilTagDetector_class.h"
#include "TagPerception_class.h"

// Built-in Cpp Libraries
#include <string>
#include <iostream>
#include <unordered_map>
#include <vector>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

// Other
#include "yaml-cpp/yaml.h"



int main(int argc, char * argv[])
{

    YAML::Node config;
    std::unordered_map<std::string, std::string> camera_dict;
    std::vector<std::string> camera_list;

    std::string dir(__FILE__);
    dir = dir.substr(0, dir.find_last_of("\\/"));
    std::string config_location = dir + "/cfg/detector_cfg.yaml";

    try {
        config = YAML::LoadFile(config_location);
        
    } catch (const YAML::BadFile& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    std::string name = config["tag_family"].as<std::string>();
    std::cout << name << std::endl;
    std::cout << "YAML file loaded successfully!" << std::endl;

    // Load in Configuration Variables
    std::string rawBackCamTopic = config["camera"]["back_cam"]["raw_cam_topic"].as<std::string>();
    std::string rawFrontCamTopic = config["camera"]["front_cam"]["raw_cam_topic"].as<std::string>();
    std::string tagFamily = config["tag_family"].as<std::string>();
    double tagSize = config["tag_size"].as<double>();

    // Initialize Node
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr perception_node = rclcpp::Node::make_shared("perception_node");

    camera_dict = {{"front_cam",rawFrontCamTopic}, {"back_cam",rawBackCamTopic}};
    camera_list = {"front_cam", "back_cam"};

    std::vector<std::unique_ptr<AprilTagDetector>> detectorObjects;
    std::vector<std::unique_ptr<TagPerception>> tagPerceptionObjects;

    for (int i=0; i< static_cast<int>(camera_dict.size()); i++){

        detectorObjects.emplace_back(std::make_unique<AprilTagDetector>(tagFamily, tagSize, camera_list[i]));
        tagPerceptionObjects.emplace_back(std::make_unique<TagPerception>(perception_node,
                                            *detectorObjects[i],
                                            camera_dict[camera_list[i]],
                                            camera_list[i]));
        std::cout << "object\n";
    }


    rclcpp::spin(perception_node);
    rclcpp::shutdown();
    return 0;
}