#include "ObjectDetectionCropper.h"

ImageCropper::ImageCropper()
        : it_(nh_)
{
    namedWindow(OPENCV_WINDOW);

    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 100,
                               &ImageCropper::imageCb, this);
    bb_sub_ = nh_.subscribe("/darknet_ros/bounding_boxes", 100, &ImageCropper::bbCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 100);
}

void ImageCropper::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
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
    if(!frame.empty())
    {
        current_frame.push_back(frame);
        current_frameId.push_back(msg->header.frame_id);
    }
}

void ImageCropper::bbCb(const darknet_ros_msgs::BoundingBoxes& msg)
{
    sensor_msgs::ImagePtr msg_send;
    if(msg.bounding_boxes.size() == 0)
        return;
    Mat curr_frame, cropped_frame, blurred_frame;
    int i, curr = -1;
    //Find if frame is stored in queue
    for(i = 0; i < current_frameId.size(); i++)
    {
        if (msg.header.frame_id == current_frameId[i])
        {
            curr = i;
            break;
        }
    }
    if(curr != -1) //If frame was found in queue
    {
        ROS_INFO("Got it!");
        current_frame[curr].copyTo(curr_frame);
        double smallest_distance= 999999;
        int smallest_id;
        //Find closest bounding box
        for(i = 0; i < msg.bounding_boxes.size(); i++)
        {
            int bb_x = (msg.bounding_boxes[i].xmin + msg.bounding_boxes[i].xmax)/2, bb_y = (msg.bounding_boxes[i].ymin + msg.bounding_boxes[i].ymax)/2;
            double euclidean_distance = sqrt(pow(bb_x-curr_frame.cols/2,2)+pow(bb_y-curr_frame.rows/2,2));
            if(euclidean_distance < smallest_distance)
            {
                smallest_distance = euclidean_distance;
                smallest_id = i;
            }
        }
        //Crop image
        Rect crop_rect(msg.bounding_boxes[smallest_id].xmin, msg.bounding_boxes[smallest_id].ymin, msg.bounding_boxes[smallest_id].xmax-msg.bounding_boxes[smallest_id].xmin,msg.bounding_boxes[smallest_id].ymax-msg.bounding_boxes[smallest_id].ymin);
        cropped_frame = curr_frame(crop_rect);
        //Blur background image
        GaussianBlur(curr_frame,blurred_frame,Size(99,99),0);
        //Copy cropped rectangle back onto blurred image
        Mat dest_roi = blurred_frame(crop_rect);
        cropped_frame.copyTo(dest_roi);
        msg_send = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cropped_frame).toImageMsg();
        image_pub_.publish(msg_send);
        imshow(OPENCV_WINDOW,blurred_frame);
        waitKey(1);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_cropper");
    ImageCropper ic;
    ros::spin();
    return 0;
}