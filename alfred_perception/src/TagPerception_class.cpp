// Project Alfred Libraries
#include "AprilTagDetector_class.h"
#include "TagPerception_class.h"

// Built-in Cpp Libraries
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.h"

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



TagPerception::TagPerception(
    rclcpp::Node::SharedPtr perceptionNode,
    AprilTagDetector& detector, 
    std::string camTopic,
    std::string camName) 
    : 
    mPerceptionNode(perceptionNode),
    mDetector(detector), 
    mRawCamTopic(std::move(camTopic)),
    mCamName(std::move(camName))
    
{
    mCamSubscription = mPerceptionNode->create_subscription<sensor_msgs::msg::Image>(
        mRawCamTopic, 10,
        std::bind(&TagPerception::imageCallback, this, std::placeholders::_1)
    );

    mtf = tag36h11_create();

    mtd = apriltag_detector_create();
    apriltag_detector_add_family(mtd, mtf);

   
}

void TagPerception::printTopic() 
{
    std::cout << "April Tag Topic: " << mRawCamTopic << "\n";
}

void TagPerception::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // std::cout << "Received an image frame!\n" << std::endl;
    // std::cout << "Message: " << msg << std::endl;

    cv::Mat image = cv_bridge::toCvCopy(msg,"bgr8")->image;

    // cv::imshow(mCamName, image);  
    // int k = cv::waitKey(1); // Wait for a keystroke in the window
    // cv::destroyWindow(mCamName);

    // Create Detector
    // apriltag_family_t *tf = NULL;
    // tf = tag36h11_create();

    // apriltag_detector_t *mtd = apriltag_detector_create();
    // apriltag_detector_add_family(mtd, tf);

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

    zarray_t *detections = apriltag_detector_detect(mtd, &im);

    

    for (int i = 0; i < zarray_size(detections); i++) {

        // Get Detection information into a readable form
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        // Populate a Structure with Dectection Information
        TagDetection_struct Detection(det);

        // Print Detection Information to the Terminal
        Detection.display();

        std::vector<std::vector<float>> cornerPoints(4, std::vector<float>(2, 0));

        
        // cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]),
        //     cv::Point(det->p[1][0], det->p[1][1]),
        //     cv::Scalar(0, 0xff, 0), 2);
        // cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]),
        //     cv::Point(det->p[3][0], det->p[3][1]),
        //     cv::Scalar(0, 0, 0xff), 2);
        // cv::line(frame, cv::Point(det->p[1][0], det->p[1][1]),
        //     cv::Point(det->p[2][0], det->p[2][1]),
        //     cv::Scalar(0xff, 0, 0), 2);
        // cv::line(frame, cv::Point(det->p[2][0], det->p[2][1]),
        //     cv::Point(det->p[3][0], det->p[3][1]),
        //     cv::Scalar(0xff, 0, 0), 2);

        // std::stringstream ss;
        // ss << det->id;
        // cv::String text = ss.str();
        // int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
        // double fontscale = 1.0;
        // int baseline;
        
        // cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
        //                                 &baseline);
        // cv::putText(frame, text, cv::Point(det->c[0]-textsize.width/2,
        //                            det->c[1]+textsize.height/2),
        //         fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
        
        
        };

        apriltag_detections_destroy(detections);

        imshow("Tag Detections", frame);
        

        

}
