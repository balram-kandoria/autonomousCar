#pragma once

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <cstdio>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>

// Other
#include "yaml-cpp/yaml.h"

// Project Alfred Libraries
#include "StaticTransform_struct.h"

class StaticTransform
{
    public:
        // Constructor
        StaticTransform(
            rclcpp::Node::SharedPtr staticTransform_node,
            YAML::Node config
        );

        // Methods
        void publishTransforms(StaticTransform_struct data);
        void getTransforms();

    private:
        // Internal Class Variables
        rclcpp::Node::SharedPtr _staticTransform_node;
        tf2_ros::StaticTransformBroadcaster _broadcaster;
        YAML::Node _config;
};