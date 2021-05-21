#include "RosTCPCommunication.h"

RosTCPCommunication::RosTCPCommunication()
{
    // Subscribe to topics
    obj_sub_ = nh_.subscribe("/object_recognition/object_name",1,&RosTCPCommunication::objCb,this);
    ori_sub_ = nh_.subscribe("/depth_data/object_orientation",1,&RosTCPCommunication::oriCb,this);
    //Init flags
    obj_received = false;
    ori_received = false;
    //TCP client socket initialization
    struct sockaddr_in serv_addr;
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        ROS_INFO("\n Socket creation error \n");
        return;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0)
    {
        ROS_INFO("\nInvalid address/ Address not supported \n");
        return;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        ROS_INFO("\nConnection Failed \n");
        return;
    }
    char *hello = "Hello from client";
    send(sock , hello , strlen(hello) , 0 );
}

void RosTCPCommunication::objCb(const std_msgs::String& msg)
{
    obj_received = true;
    obj = msg.data;
    if(ori_received)
        sendTCP();
}

void RosTCPCommunication::oriCb(const std_msgs::Float32& msg)
{
    ori_received = true;
    ori = msg.data;
    if(obj_received)
        sendTCP();,
}

void RosTCPCommunication::sendTCP()
{
    string message = obj + " " + to_string(ori);
    send(sock , message.c_str() , strlen(message.c_str()) , 0 );
    obj_received = ori_received = false;
    ROS_INFO(message.c_str());
    ROS_INFO("Message was sent.");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tcp_communication");
    RosTCPCommunication ros_tcp;
    ros::spin();
    return 0;
}