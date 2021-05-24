//
// Created by silverback on 5/24/21.
//

#ifndef ROB8_WS_OBJECTDETECTIONSTAMP_H
#define ROB8_WS_OBJECTDETECTIONSTAMP_H

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;

class ImageStamper
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
public:
    ImageStamper();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
};

#endif //ROB8_WS_OBJECTDETECTIONSTAMP_H
