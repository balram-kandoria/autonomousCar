#include <iostream>
#include <string>
#include <vector>
#include <variant>
#include <unordered_map>

#include "TagDetection_struct.h"

using VariantType = std::variant<int, float, std::vector<std::vector<float>>, std::vector<float>>;

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
        std::cout << "\t[" << cornerPoints[i][0] << ", " << cornerPoints[i][1] << "]\n";
    }
    std::cout << "\t}\n";

}

std::unordered_map<std::string, VariantType> TagDetection_struct::returnDetection() {
    
    std::unordered_map<std::string, VariantType> Detection;

    Detection["id"] = id;
    Detection["hamming"] = hamming;
    Detection["decision_margin"] = decision_margin;
    Detection["centerPoints"] = centerPoints;
    Detection["cornerPoints"] = cornerPoints;

    return Detection;
}