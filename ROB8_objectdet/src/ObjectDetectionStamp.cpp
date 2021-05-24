#include "ObjectDetectionStamp.h"

ImageStamper::ImageStamper()
        : it_(nh_)
{
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/color/image_raw", 100,
                               &ImageStamper::imageCb, this);
    image_pub_ = it_.advertise("/camera/color/image_stamped", 100);
}

void ImageStamper::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::ImagePtr msg_send;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Mat frame;
    frame = cv_ptr->image;
    msg_send = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    msg_send->header.stamp = ros::Time::now();
    image_pub_.publish(msg_send);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_stamper");
    ImageStamper ic;
    ros::spin();
    return 0;
}