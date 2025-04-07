// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/LinearMath/Quaternion.h>

// Built-in Cpp Libraries
#include <string>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <cstdio>

// Other
#include "yaml-cpp/yaml.h"

// Project Alfred Libraries
#include "staticTransform_class.h"
#include "StaticTransform_struct.h"

StaticTransform::StaticTransform(
    rclcpp::Node::SharedPtr staticTransform_node,
    YAML::Node config)
    :
    _staticTransform_node(staticTransform_node),
    _broadcaster(staticTransform_node),
    _config(config)
{
    std::cout << "Initialized Static Transform\n";
}

void StaticTransform::publishTransforms(StaticTransform_struct data)
{
    geometry_msgs::msg::TransformStamped static_transformStamped;
    rclcpp::Time now = _staticTransform_node->get_clock()->now();
    static_transformStamped.header.stamp = now;
    static_transformStamped.header.frame_id = data.frame_id;
    static_transformStamped.child_frame_id = data.child_frame_id;
    static_transformStamped.transform.translation.x = data.translation[0];
    static_transformStamped.transform.translation.y = data.translation[1];
    static_transformStamped.transform.translation.z = data.translation[2];

    tf2::Quaternion quat;
    quat.setRPY(data.rotation[0], data.rotation[1], data.rotation[2]);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    _broadcaster.sendTransform(static_transformStamped);

}

void StaticTransform::getTransforms()
{

    StaticTransform_struct TransformStruct_Object(_config["static_transforms"]["frame1"]);
    int numStaticTransforms = _config["static_transforms"].size();

    for (int i = 0; i<numStaticTransforms; i++){

        std::stringstream ss;
        ss << "frame" << i+1;
        std::string frameValue = ss.str();
        
        std::cout << "Grabbing " << frameValue << " data from the static transforms list ... \n";

        TransformStruct_Object.updateValues(_config["static_transforms"][frameValue]);

        publishTransforms(TransformStruct_Object);
        std::cout << "Publishing " << frameValue << " data ... \n";

    };
    

    std::cout << TransformStruct_Object.translation[2] << "\n";
    // std::cout << StaticTransform_List.size() << "\n";

};