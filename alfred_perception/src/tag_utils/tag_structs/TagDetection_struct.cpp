#include <iostream>
#include <string>
#include <vector>
#include <variant>
#include <unordered_map>

// Other Libraries
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>


#include "TagDetection_struct.h"

// using VariantType = std::variant<int, float, std::vector<std::vector<float>>, std::vector<cv::Point2d>;

void TagDetection_struct::display() const {

    std::cout << "Dection Information\n";
    std::cout << "ID: " << id << "\n";
    std::cout << "Hamming: " << hamming << "\n";
    std::cout << "Decision Margin: " << decision_margin << "\n";

    std::cout << "Center Points: {\n";
    // Print Center Information
    for (int i=0; i<1; i++){
        std::cout << "\t[" << centerPoints[i] << ", " << centerPoints[i+1] << "]\n";
    }
    std::cout << "\t}\n";

    std::cout << "Corner Points: {\n";
    // Print Center Information
    for (int i=0; i<4; i++){
        std::cout << "\t[" << cornerPoints[i] << "]\n";
    }
    std::cout << "\t}\n";

}

cv::Mat TagDetection_struct::drawDetection(cv::Mat img) {
    cv::Scalar green = cv::Scalar(0,255,0,0);
    int thickness = 3;

    for (size_t i = 0; i < cornerPoints.size(); i++) {
        if (i != cornerPoints.size()-1) {
            cv::line(img, cornerPoints[i], cornerPoints[i+1], green, thickness);
        }
        else {
            cv::line(img, cornerPoints[i], cornerPoints[0], green, thickness);
        }
    };

    std::string ID = std::to_string(id);
    double imgCenterX = abs((cornerPoints[0].x - cornerPoints[1].x) / 2.0) + abs(cornerPoints[0].x); 
    double imgCenterY = abs((cornerPoints[1].y - cornerPoints[2].y) / 2.0) + abs(cornerPoints[2].y); 

    std::cout << imgCenterX << " " << imgCenterY << "\n";

    cv::Point origin = cv::Point2d(imgCenterX, imgCenterY);
    int font = cv::FONT_HERSHEY_SIMPLEX;
    double fontSize = 1.0;
    cv::Scalar blue = cv::Scalar(255,0,0,0);

    cv::putText(img, ID, origin, font, fontSize, blue, thickness);

    return img;
}