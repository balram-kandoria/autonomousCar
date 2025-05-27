#pragma once

#include "rclcpp/rclcpp.hpp"

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <geometry_msgs/msg/pose.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>



class worldMap
{
    public:
        // Constructor
        worldMap(
            rclcpp::Node::SharedPtr mapNode,
            std::string odom_topic,
            std::string lidar_topic);
        
        // Methods
        void updateVehiclePosition();
        void initWorldMap();
        void updateMap();

    private:
        rclcpp::Node::SharedPtr _mapNode;
        std::string _odom_topic;
        std::string _lidar_topic;
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomSub;
        // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laserscanSub;
        std::vector<rclcpp::SubscriptionBase::SharedPtr> _subcriptionsList;

        std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> _odomSub;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> _laserscanSub;
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>>> _approxTimeSync;

        // Callbacks
        void SyncCallback(const std::shared_ptr<nav_msgs::msg::Odometry const>& odom_msg,
            const std::shared_ptr<sensor_msgs::msg::LaserScan const>& scan_msg);
};

