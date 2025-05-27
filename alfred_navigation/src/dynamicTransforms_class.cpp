
#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_ros/buffer.h"



// Other
#include "yaml-cpp/yaml.h"


// Alfred Libraries
#include "dynamicTransforms_class.h"
#include "alfred_interfaces/msg/april_tag_detection.hpp"


DynamicTransform::DynamicTransform(
    rclcpp::Node::SharedPtr dynamicTransform_node,
    std::string subcriptionTopic)
    :
    _dynamicTransform_node(dynamicTransform_node),
    _dyanmicBroadcastor(dynamicTransform_node),
    _subcriptionTopic(subcriptionTopic),
    _buffer(_dynamicTransform_node->get_clock()),
    _listener(_buffer)
    {
        _TagDetectionSubscription = _dynamicTransform_node->create_subscription<alfred_interfaces::msg::AprilTagDetection>(
            _subcriptionTopic, 10,
            std::bind(&DynamicTransform::transformCallback, this, std::placeholders::_1)
        );
    }

void DynamicTransform::transformCallback(const alfred_interfaces::msg::AprilTagDetection::SharedPtr msg)
{
    // geometry_msgs::msg::TransformStamped transformStamped;
    geometry_msgs::msg::PoseStamped poseMsg;

    rclcpp::Time now = _dynamicTransform_node->get_clock()->now();

    poseMsg.header.stamp = now;
    poseMsg.header.frame_id = msg->header.frame_id;
    poseMsg.pose.position.x = msg->x;
    poseMsg.pose.position.y = msg->y;
    poseMsg.pose.position.z = msg->z;

    tf2::Quaternion q;
    q.setRPY(msg->roll, msg->pitch, msg->yaw);
    poseMsg.pose.orientation.x = q.x();
    poseMsg.pose.orientation.y = q.y();
    poseMsg.pose.orientation.z = q.z();
    poseMsg.pose.orientation.w = q.w();

    
    // std::cout << "Camera RPY: \n" << msg->roll << "\n" << msg->pitch << "\n" << msg->yaw << "\n";

    geometry_msgs::msg::PoseStamped tranformedPose = _buffer.transform(poseMsg,"base_link");

    double roll, pitch, yaw;

    // Convert quaternion to RPY (yaw-pitch-roll order is default)
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // std::cout << "Base Link RPY: \n" << roll<< "\n" << pitch << "\n" << yaw << "\n";

    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = now;
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = tranformedPose.header.frame_id;
    transformStamped.transform.translation.x = -1*tranformedPose.pose.position.x;
    transformStamped.transform.translation.y = -1*tranformedPose.pose.position.y;
    transformStamped.transform.translation.z = -1*tranformedPose.pose.position.z;

    transformStamped.transform.rotation.x = poseMsg.pose.orientation.x;
    transformStamped.transform.rotation.y = poseMsg.pose.orientation.y;
    transformStamped.transform.rotation.z = poseMsg.pose.orientation.z;
    transformStamped.transform.rotation.w = poseMsg.pose.orientation.w;

    _dyanmicBroadcastor.sendTransform(transformStamped);

}