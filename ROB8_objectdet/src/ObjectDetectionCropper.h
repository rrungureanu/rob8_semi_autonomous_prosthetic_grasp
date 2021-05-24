//
// Created by silverback on 4/23/21.
//

#ifndef SRC_OBJECTDETECTIONCROPPER_H
#define SRC_OBJECTDETECTIONCROPPER_H

#include <vector>
#include <math.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Cropped image";

class ImageCropper
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Subscriber bb_sub_;
    vector<Mat> current_frame;
    vector<ros::Time> current_frameStamp;
public:
    ImageCropper();
    ~ImageCropper()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void bbCb(const darknet_ros_msgs::BoundingBoxes& msg);
};
#endif //SRC_OBJECTDETECTIONCROPPER_H
