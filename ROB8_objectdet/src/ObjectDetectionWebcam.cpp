#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/color/image_raw", 1);
    ROS_INFO("B4 capture");
    cv::VideoCapture cap(0);
    // Check if video device can be opened with the given index
    if(!cap.isOpened()) return 1;
    ROS_INFO("Camera opened");
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    int frame_id = 1;
    ros::Rate loop_rate(5);
    while (nh.ok()) {
        cap >> frame;
        // Check if grabbed frame is actually full with some content
        if(!frame.empty()) {
            imshow("Full frame",frame);
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            msg->header.stamp = ros::Time::now();
            msg->header.frame_id = std::to_string(frame_id);
            pub.publish(msg);
            cv::waitKey(1);
            frame_id++;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}