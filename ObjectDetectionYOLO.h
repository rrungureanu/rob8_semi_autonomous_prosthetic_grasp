//
// Created by silverback on 3/28/21.
//

#ifndef SRC_OBJECTDETECTIONYOLO_H
#define SRC_OBJECTDETECTIONYOLO_H

#include <fstream>
#include <sstream>
#include <iostream>
#include <math.h>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace dnn;
using namespace std;

// Initialize the parameters
float confThreshold = 0.5; // Confidence threshold
float nmsThreshold = 0.4;  // Non-maximum suppression threshold
int inpWidth = 320;  // Width of network's input image
int inpHeight = 320; // Height of network's input image
vector<string> classes;
static const std::string OPENCV_WINDOW = "Image window";

// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess(Mat& frame, const vector<Mat>& outs, Rect& out);

// Get the names of the output layers
vector<String> getOutputsNames(const Net& net);

class ImageConverter
{
    Net net;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    ImageConverter();
    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
};

#endif //SRC_OBJECTDETECTIONYOLO_H
