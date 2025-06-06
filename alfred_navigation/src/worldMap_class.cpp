
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "worldMap_class.h"

using std::placeholders::_1;
using std::placeholders::_2;

worldMap::worldMap(
    rclcpp::Node::SharedPtr mapNode,
    std::string odom_topic,
    std::string lidar_topic)
    :
    _mapNode(mapNode),
    _odom_topic(odom_topic),
    _lidar_topic(lidar_topic)
    {
        rclcpp::QoS qos = rclcpp::QoS(10);
        rmw_qos_profile_t raw_qos = qos.get_rmw_qos_profile();
        auto base_interface = _mapNode->get_node_base_interface();
        auto topics_interface = _mapNode->get_node_topics_interface();

        _odomSub = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>();
        _laserscanSub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>();

        _odomSub->subscribe(_mapNode, _odom_topic, raw_qos);

        _laserscanSub->subscribe(_mapNode, _lidar_topic, raw_qos);

        uint32_t queue_size = 10;

        _approxTimeSync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::
                            ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>>>(
                            message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry,
                            sensor_msgs::msg::LaserScan>(queue_size), *_odomSub, *_laserscanSub);

        _approxTimeSync->setAgePenalty(0.50);
        _approxTimeSync->registerCallback(std::bind(&worldMap::SyncCallback, this, _1, _2));
    }

void worldMap::SyncCallback(const std::shared_ptr<nav_msgs::msg::Odometry const>& odom_msg,
    const std::shared_ptr<sensor_msgs::msg::LaserScan const>& scan_msg)
{

    std::vector<float> rangeData = scan_msg->ranges;
    double max = *std::max_element(rangeData.begin(), rangeData.end());
    // std::vector<float>  maxRange = std::abs(rangeData);

    std::cout << max << "\n";

    // for (size_t i=0; i < rangeData.size(); i++)
    // {
    //     std::cout << (rangeData[i]) << "\n";
    // };
    
    std::cout << "Here" << "\n";
}