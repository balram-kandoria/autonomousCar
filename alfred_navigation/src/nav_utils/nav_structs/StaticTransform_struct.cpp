// Built-in Cpp Libraries
#include <string>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <cstdio>

// Other
#include "yaml-cpp/yaml.h"

// Project Alfred Libraries
#include "StaticTransform_struct.h"


void StaticTransform_struct::updateValues(YAML::Node config) 
{
    frame_id = config["frame_id"].as<std::string>();
    child_frame_id = config["child_frame_id"].as<std::string>();
    translation = config["translation"].as<std::vector<float>>();
    rotation = config["rotation"].as<std::vector<float>>();
}