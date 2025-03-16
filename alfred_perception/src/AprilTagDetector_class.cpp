#include "AprilTagDetector_class.h"

// Built-in Cpp Libraries
#include <memory>
#include <string>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

// Other Libraries
#include "apriltag/apriltag.h"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

// Structures
#include "TagDetection_struct.h"

// C Libraries
extern "C" {
    #include "apriltag/apriltag.h"
    #include "apriltag/apriltag_pose.h"
    #include "apriltag/tag36h11.h"
    #include "apriltag/tag25h9.h"
    #include "apriltag/tag16h5.h"
    #include "apriltag/tagCircle21h7.h"
    #include "apriltag/tagCircle49h12.h"
    #include "apriltag/tagCustom48h12.h"
    #include "apriltag/tagStandard41h12.h"
    #include "apriltag/tagStandard52h13.h"
    #include "apriltag/common/getopt.h"
    }

using VariantType = std::variant<int, float, std::vector<std::vector<float>>, std::vector<float>>;

AprilTagDetector::AprilTagDetector(
    std::string TagFamily, 
    double TagSize,
    std::string camName):
    mCamName(std::move(camName)) 
{
    mTagFamily = TagFamily;
    mTagSize = TagSize;

    // Initialize and Set Tag Family
    apriltag_family_t *mtf = NULL;
    if (mTagFamily == "tag36h11") {
        mtf = tag36h11_create();
    };
    
    // Create Detector
    mtd = apriltag_detector_create();
    apriltag_detector_add_family(mtd, mtf);
}

void AprilTagDetector::printFamily() 
{
    std::cout << "April Tag Family: " << mTagFamily << "\n";
}

std::unordered_map<std::string, VariantType> AprilTagDetector::detectTag(const cv::Mat &image)
{
    std::cout << "Running Tag Detector" << "\n";

    // Create Empty Mat items to store manipulated images
    cv::Mat frame, gray;

    // Create a copy of image to frame for visualizing detections
    frame = image;

    // Add gray version of frame into variable gray
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // Show gray image
    cv::imshow(mCamName, gray);  
    cv::waitKey(1); 

    // Make an image_u8_t header for the Mat data
    image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};

    std::cout << "1\n";
    // Perform Detection of April Tags
    zarray_t *detections = apriltag_detector_detect(mtd, &im); 
    std::cout << "2\n";   

    std::unordered_map<std::string, VariantType>  Detection;

    std::cout << "3\n"; 

    for (int i = 0; i < zarray_size(detections); i++) {
        
        std::cout << "4\n"; 
        // Get Detection information into a readable form
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        std::cout << "5\n"; 

        // Populate a Structure with Dectection Information
        TagDetection_struct Detection_object(det);

        std::cout << "6\n"; 

        // Print Detection Information to the Terminal
        // Detection_object.display();

        Detection = Detection_object.returnDetection();

        std::vector<std::vector<float>> cornerPoints(4, std::vector<float>(2, 0));
    };

    // Empty Detections Variable
    apriltag_detections_destroy(detections);

    // Show Tag Image with Detection Overlay
    imshow(mCamName + " Tag Detections", frame);

    std::cout << "End detect callback\n";

    return Detection;
}