#ifndef STATICTRANSFORM_STRUCT_H
#define STATICTRANSFORM_STRUCT_H

// Built-in Cpp Libraries
#include <string>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <cstdio>

// Other
#include "yaml-cpp/yaml.h"

struct StaticTransform_struct{
    // Define Default Variable Values
    YAML::Node frame;
    std::string frame_id = "default_parent_frame_id";
    std::string child_frame_id = "default_child_frame_id";
    std::vector<float> translation = {0,0,0}; // x, y, z
    std::vector<float> rotation = {0,0,0}; // roll, pitch, yaw

    // Define Constructor
    StaticTransform_struct(YAML::Node config={})
    : frame(config) 
    {
        frame_id = frame["frame_id"].as<std::string>();
        child_frame_id = frame["child_frame_id"].as<std::string>();
        translation = frame["translation"].as<std::vector<float>>();
        rotation = frame["rotation"].as<std::vector<float>>();
    }

    void updateValues(YAML::Node config);
};



#endif // STATIC_TRANSFORM_STRUCT_H