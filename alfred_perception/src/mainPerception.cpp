#include "AprilTagDetector_class.h"
#include <string>

int main()
{

    std::string tagFamily = "Tag36h11";
    AprilTagDetector Tag{"Tag36h11", 0.150};

    Tag.printFamily();

    return 0;
};