#pragma once

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"

// Alfred Libraries
#include "alfred_interfaces/msg/april_tag_detection.hpp"



// Other
#include "yaml-cpp/yaml.h"

class DynamicTransform
{
    public:
        //Constructor
        DynamicTransform(
            rclcpp::Node::SharedPtr dynamicTransform_node,
            std::string subcriptionTopic
        );

        void publishTransforms();

    private:
        // Internal Variables
        rclcpp::Node::SharedPtr _dynamicTransform_node;
        tf2_ros::TransformBroadcaster _dyanmicBroadcastor;
        rclcpp::Subscription<alfred_interfaces::msg::AprilTagDetection>::SharedPtr _TagDetectionSubscription;
        std::string _subcriptionTopic;
        tf2_ros::Buffer _buffer;
        tf2_ros::TransformListener _listener;
        

        // Callbacks
        void transformCallback(const alfred_interfaces::msg::AprilTagDetection::SharedPtr msg);
        
};