// Project Alfred Libraries
#include "worldTransforms_class.h"
#include "staticTransform_class.h"
#include "dynamicTransforms_class.h"
#include "worldMap_class.h"

// Other
#include "yaml-cpp/yaml.h"

// Built-in Cpp Libraries
#include <string>
#include <iostream>
#include <unordered_map>
#include <vector>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"




int main(int argc, char * argv[])
{

    YAML::Node config;

    std::string dir(__FILE__);
    dir = dir.substr(0, dir.find_last_of("\\/"));
    std::string config_location = dir + "/cfg/navigation_cfg.yaml";

    try {
        config = YAML::LoadFile(config_location);
        
    } catch (const YAML::BadFile& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    std::cout << "YAML file loaded successfully!" << std::endl;

    // Initialize Node
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr transform_node = rclcpp::Node::make_shared("transform_node");

    std::unique_ptr<StaticTransform> StaticBroadcastor;
    std::unique_ptr<DynamicTransform> DynamicBroadcastor;
    std::unique_ptr<worldMap> MapObject;

    StaticBroadcastor = std::make_unique<StaticTransform>(transform_node,config);

    DynamicBroadcastor = std::make_unique<DynamicTransform>(transform_node,"/vehicle/sensor/front_cam/tagdetection");

    MapObject = std::make_unique<worldMap>(transform_node,"/vehicle/odometry","/vehicle/sensor/front_lidar/scan");

    // StaticBroadcastor->publishTransforms();
    StaticBroadcastor->getTransforms();

    rclcpp::spin(transform_node);
    rclcpp::shutdown();
    return 0;
}