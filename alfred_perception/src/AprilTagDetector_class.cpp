#include "AprilTagDetector_class.h"
#include <iostream>

AprilTagDetector::AprilTagDetector(std::string TagFamily, double TagSize) 
{
    mTagFamily = TagFamily;
    mTagSize = TagSize;
}

void AprilTagDetector::printFamily() 
{
    std::cout << "April Tag Family: " << mTagFamily << "\n";
}