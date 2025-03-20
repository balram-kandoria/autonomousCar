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
#include <opencv2/calib3d.hpp>

// Structures
#include "TagDetection_struct.h"

// Custom Libraries
#include "tag_helper.h"

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
    std::string camName,
    cv::Mat camDistortionCoeff,
    cv::Mat camMatrix,
    bool visualize,
    bool debug):
    _CamName(std::move(camName)) 
{
    _TagFamily = TagFamily;
    _TagSize = TagSize;
    _camDistortionCoeff = camDistortionCoeff;
    _camMatrix = camMatrix;
    _visualize = visualize;
    _debug = debug;

    double halfTagDist = TagSize/2;

    _objPt = (cv::Mat1d(4,3) << -halfTagDist, halfTagDist, 0, halfTagDist, halfTagDist, 0, halfTagDist, -halfTagDist, 0, -halfTagDist, -halfTagDist, 0);
    
    // Initialize and Set Tag Family
    apriltag_family_t *_tf = NULL;
    if (_TagFamily == "tag36h11") {
        _tf = tag36h11_create();
    };
    
    // Create Detector
    _td = apriltag_detector_create();
    apriltag_detector_add_family(_td, _tf);
}

void AprilTagDetector::printFamily() 
{
    std::cout << "April Tag Family: " << _TagFamily << "\n";
}

std::vector<TagDetection_struct> AprilTagDetector::detectTag(const cv::Mat &image)
{
    std::cout << "Running Tag Detector" << "\n";

    // Create Empty Mat items to store manipulated images
    cv::Mat frame, gray;    

    // Create a copy of image to frame for visualizing detections
    frame = image;

    // Add gray version of frame into variable gray
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // Make an image_u8_t header for the Mat data
    image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};
    
    // Perform Detection of April Tags
    zarray_t *detections = apriltag_detector_detect(_td, &im); 
    
    std::unordered_map<std::string, VariantType>  Detection;
    
    std::vector<TagDetection_struct> detection_list;

    apriltag_detection_t *det;
    
    for (int i = 0; i < zarray_size(detections); i++) {
 
        // Get Detection information into a readable form
        
        zarray_get(detections, i, &det);
        
        

        // Populate a Structure with Dectection Information
        TagDetection_struct Detection_object(det);

        cv::Mat rvec, tvec;

        std::cout << _objPt <<"\n";

        std::cout << Detection_object.cornerPoints << "\n";

        std::cout << _camMatrix << "\n";

        std::cout << _camDistortionCoeff << "\n";
        
        bool success = cv::solvePnPRansac(_objPt, Detection_object.cornerPoints, _camMatrix, _camDistortionCoeff, rvec, tvec);

        
        Detection_object.x = 4.0;
        
        if (_debug) {
            // Print Detection Information to the Terminal
            Detection_object.display();
        };
        
        detection_list.emplace_back(Detection_object);

        // Detection = Detection_object.returnDetection();

        std::cout << Detection_object.centerPoints[0] << "\n";

    };

    // Empty Detections Variable
    apriltag_detections_destroy(detections);

    if (_visualize) {
        // Show gray image
        cv::imshow(_CamName, gray);
        
        cv::Mat frame_w_detection = add_Detection_to_Image(det, frame);

        // Show Tag Image with Detection Overlay
        imshow(_CamName + " Tag Detections", frame_w_detection);

        cv::waitKey(10); 
    };


    return detection_list;
}