
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
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

        _resolution = 0.01;

        _mapPublisher = _mapNode->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "map_topic", 10
        );

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
    double range_max = scan_msg->range_max;
    
    // std::vector<float>  maxRange = std::abs(rangeData);

    double min = *std::min_element(rangeData.begin(), rangeData.end());

    for (size_t i=0; i < rangeData.size(); i++)
    {
        if (rangeData[i] >= range_max)
            {
                rangeData[i] = -1.0;
            };

        std::cout << rangeData[i] << "\n";
    };
    
    std::cout << "Here" << "\n";

    double max = *std::max_element(rangeData.begin(), rangeData.end());

    double rounded_max = std::round(max*100)/100;


    // Initialize Angle parameters
    double angle= scan_msg->angle_min;
    double angle_inc = scan_msg->angle_increment;

    int cols = -1;
    int rows = cols;
    // Initialize a Matrix with -1
    if ((static_cast<int>(rounded_max*100) % 2) == 0){
        cols = rounded_max*200 + 1;
        rows = cols;
    }
    else{
        cols = rounded_max*100*2;
        rows = cols;
    }
    std::vector<std::vector<int>> matrix(rows, std::vector<int>(cols, -1));

    // Define the center coordinates of the measurment / matrix
    int centerY = static_cast<int>(std::round(static_cast<double>(cols) / 2.0));
    int centerX = centerY;

    for (size_t i=0; i < rangeData.size(); i++)
    {
        if (rangeData[i] != -1)
        {
            double current_angle = angle + (angle_inc*i);
            int x = static_cast<int>((100 * (rangeData[i]) * std::cos(current_angle))+rounded_max*100);
            int y = static_cast<int>((100 * (rangeData[i]) * std::sin(current_angle))+rounded_max*100);

            // std::cout << "(" << x << ", " << y << ")\n";

            matrix[x][y] = 100;
            if (x < 483 || y < 483)
            {
                std::cout << "indexError: "<<x<<", "<<y << "\n";
            }
        }

    }

    std::cout << "Matrix Center: " << centerX << " " << centerY << "\n";
    std::cout << "Matrix Size: " << cols << " " << rows << "\n";

    std::cout << min << "\n";
    std::cout << rounded_max << "\n";

    std::vector<signed char> data = {};
    for (int i=0; i < cols; i++)
    {
        for (int j=0; j < cols; j++)
        {
            std::cout << matrix[i][j] << " ";
            data.emplace_back(matrix[i][j]);
        }
        std::cout << "\n";
    }

    nav_msgs::msg::OccupancyGrid mapMsg;

    rclcpp::Time now = _mapNode->get_clock()->now();

    mapMsg.header.stamp = now;
    mapMsg.header.frame_id = "base_link";

    mapMsg.info.resolution = 0.01;
    mapMsg.info.width = rows;
    mapMsg.info.height = cols;

    mapMsg.info.origin.position.x = 0.0;
    mapMsg.info.origin.position.y = 0.0;
    mapMsg.info.origin.position.z = 0.0;


    mapMsg.info.origin.orientation.x = 0.0;
    mapMsg.info.origin.orientation.y = 0.0;
    mapMsg.info.origin.orientation.z = 0.0;
    mapMsg.info.origin.orientation.w = 1.0;

    mapMsg.data = data;

    _mapPublisher->publish(mapMsg);

    // for (int i=0; i < cols; i++)
    // {
    //     for (int j=0; j < cols; j++)
    //     {
    //         std::cout << matrix[i][j];
    //     }
    //     std::cout << "\n";
    // }


}