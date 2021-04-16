#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);
    ROS_INFO("B4 capture");
    cv::VideoCapture cap("/home/silverback/ROB8_ws/src/ROB8_objectdet/src/run.mp4");
    // Check if video device can be opened with the given index
    if(!cap.isOpened()) return 1;
    ROS_INFO("Camera opened");
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(5);
    while (nh.ok()) {
        cap >> frame;
        // Check if grabbed frame is actually full with some content
        if(!frame.empty()) {
            imshow("Full frame",frame);
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
            cv::waitKey(1);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}