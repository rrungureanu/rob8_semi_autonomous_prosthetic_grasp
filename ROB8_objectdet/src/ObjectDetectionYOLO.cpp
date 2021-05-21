#include "ObjectDetectionYOLO.h"

//Return object box to which the frame needs to be cropped
void postprocess(Mat& frame, const vector<Mat>& outs, Rect& out)
{
    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;

    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(Rect(left, top, width, height));
            }
        }
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with lower confidences
    vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    //Find box which has its center closest to the image center
    double smallestDistance = 100000;
    int smallestId = -1;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        Rect box = boxes[idx];
        double euclideanDistance = sqrt(pow(box.x+box.width-frame.cols/2,2)+pow(box.y+box.height-frame.rows/2,2));
        if(euclideanDistance<smallestDistance)
        {
            smallestDistance = euclideanDistance;
            smallestId = idx;
        }
    }
    if(smallestId != -1)
        out = boxes[smallestId];
    else
        out.width = -1; //flagging it as failed, image will not be cropped if no detections are present
}

// Get the names of the output layers
vector<String> getOutputsNames(const Net& net)
{
    static vector<String> names;
    ROS_INFO("Getting output names");
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    ROS_INFO("Got output names");
    return names;
}

ImageConverter::ImageConverter()
        : it_(nh_)
{
    namedWindow(OPENCV_WINDOW);
    namedWindow("blob");
    cap.open("/home/silverback/ROB8_ws/src/ROB8_objectdet/src/run.mp4");
    // Load names of classes
    string classesFile = "/home/silverback/ROB8_ws/src/ROB8_objectdet/src/coco.names";
    ifstream ifs(classesFile.c_str());
    string line;
    while (getline(ifs, line)) classes.push_back(line);

    // Get the configuration and weight files for the model
    String modelConfiguration = "/home/silverback/ROB8_ws/src/ROB8_objectdet/src/yolov3.cfg";
    String modelWeights = "/home/silverback/ROB8_ws/src/ROB8_objectdet/src/yolov3.weights";

    net = readNetFromDarknet(modelConfiguration, modelWeights);

    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
                               &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
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
    Mat frame, blob;
    //frame = cv_ptr->image;
    cap>>frame;
    if(!frame.empty())
    {
        ROS_INFO("Got here1");
        // Process image
        // Create a 4D blob from a frame.
        blobFromImage(frame, blob, 1 / 255.0, cv::Size(inpWidth, inpHeight), Scalar(0, 0, 0), true, false);
        imshow(OPENCV_WINDOW,frame);
        ROS_INFO("Got here2");
        //Sets the input to the network
        net.setInput(blob);
        ROS_INFO("Got here3");
        // Runs the forward pass to get output of the output layers
        vector<Mat> outs;
        net.forward(outs, getOutputsNames(net));
        ROS_INFO("Got here4");
        // Remove the bounding boxes with low confidence and return most central box
        Rect centralBox;
        postprocess(frame, outs, centralBox);
        Mat croppedImage = frame(centralBox);
        ROS_INFO("Got here5");
        cv_ptr->image = croppedImage;
        imshow(OPENCV_WINDOW, croppedImage);
        //Publish cropped image
        image_pub_.publish(cv_ptr->toImageMsg());
        waitKey(1);
    }
}

int main(int argc, char** argv)
{
ros::init(argc, argv, "image_converter");
ImageConverter ic;
ros::spin();
return 0;
}