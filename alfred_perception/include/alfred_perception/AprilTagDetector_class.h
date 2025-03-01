#pragma once
#include <string>

class AprilTagDetector
{
    public: 
        // Constructor
        AprilTagDetector(std::string TagFamily, double TagSize);

        // Methods
        void printFamily();

    private:
        std::string mTagFamily;
        double mTagSize;
        
};