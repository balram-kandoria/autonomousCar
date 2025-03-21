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
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

void printPoints3D(const std::vector<cv::Point3d>& points) {
    std::cout << "3D Points: [";
    for (size_t i = 0; i < points.size(); i++) {
        std::cout << "(" << points[i].x << ", " << points[i].y << ", " << points[i].z << ")";
        if (i < points.size() - 1) std::cout << ", ";  // Add comma for separation
    }
    std::cout << "]\n";
}

int main(int argc, char * argv[])
{

    YAML::Node config;
    std::unordered_map<std::string, std::string> camera_dict;
    std::vector<std::string> camera_list;
    std::vector<cv::Mat> cameraMatrix_list;
    std::vector<cv::Mat> distortionCoeffs_list;
    std::vector<bool> debug_list;
    std::vector<bool> visualize_list;

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

    // // OpenCV Camera Parameters

    // TODO: use cv::FileStorage fs("calib.yml", cv::FileStorage::READ); to read yaml data
    std::vector<double> tempDistortionCoeff = config["camera"]["back_cam"]["d"].as<std::vector<double>>();

    cv::Mat backCamDistortionCoeff = (cv::Mat1d(1,5) << tempDistortionCoeff[0], tempDistortionCoeff[1], tempDistortionCoeff[2], tempDistortionCoeff[3], tempDistortionCoeff[4]);
    tempDistortionCoeff = config["camera"]["front_cam"]["d"].as<std::vector<double>>();
    cv::Mat frontCamDistortionCoeff = (cv::Mat1d(1,5) << tempDistortionCoeff[0], tempDistortionCoeff[1], tempDistortionCoeff[2], tempDistortionCoeff[3], tempDistortionCoeff[4]);

    std::cout << frontCamDistortionCoeff << "\n";
    // delete &tempDistortionCoeff; 
    // tempDistortionCoeff = nullptr;

    std::vector<std::vector<double>> tempCamMatrix = config["camera"]["back_cam"]["k"].as<std::vector<std::vector<double>>>();

    cv::Mat backCamMatrix = (cv::Mat1d(3,3) << tempCamMatrix[0][0], tempCamMatrix[0][1], tempCamMatrix[0][2], tempCamMatrix[1][0], tempCamMatrix[1][1], tempCamMatrix[1][2], tempCamMatrix[2][0], tempCamMatrix[2][1], tempCamMatrix[2][2]);
    tempCamMatrix = config["camera"]["front_cam"]["k"].as<std::vector<std::vector<double>>>();
    cv::Mat frontCamMatrix = (cv::Mat1d(3,3) << tempCamMatrix[0][0], tempCamMatrix[0][1], tempCamMatrix[0][2], tempCamMatrix[1][0], tempCamMatrix[1][1], tempCamMatrix[1][2], tempCamMatrix[2][0], tempCamMatrix[2][1], tempCamMatrix[2][2]);

    std::cout << frontCamMatrix << "\n";
    // delete &tempCamMatrix; 
    // tempCamMatrix = nullptr;

    // Initialize Node
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr perception_node = rclcpp::Node::make_shared("perception_node");

    // TODO: Create a large dictionary to hold all the below information
    camera_dict = {{"front_cam",rawFrontCamTopic}, {"back_cam",rawBackCamTopic}};
    camera_list = {"front_cam", "back_cam"};
    cameraMatrix_list = {frontCamMatrix, backCamMatrix};
    distortionCoeffs_list = {frontCamDistortionCoeff, backCamDistortionCoeff};
    debug_list = {true, false};
    visualize_list = {true, false};

    std::vector<std::unique_ptr<AprilTagDetector>> detectorObjects;
    std::vector<std::unique_ptr<TagPerception>> tagPerceptionObjects;

    for (int i=0; i< static_cast<int>(camera_dict.size()); i++){

        detectorObjects.emplace_back(std::make_unique<AprilTagDetector>(tagFamily, 
                                                                        tagSize, 
                                                                        camera_list[i], 
                                                                        distortionCoeffs_list[i],
                                                                        cameraMatrix_list[i],
                                                                        visualize_list[i], 
                                                                        debug_list[i]));

        tagPerceptionObjects.emplace_back(std::make_unique<TagPerception>(perception_node,
                                            *detectorObjects[i],
                                            camera_dict[camera_list[i]],
                                            camera_list[i]));
    }


    rclcpp::spin(perception_node);
    rclcpp::shutdown();
    return 0;
}