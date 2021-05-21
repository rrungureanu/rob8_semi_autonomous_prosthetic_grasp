//
// Created by silverback on 5/19/21.
//

#ifndef ROB8_WS_ROSTCPCOMMUNICATION_H
#define ROB8_WS_ROSTCPCOMMUNICATION_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#define PORT 30002

using namespace std;

class RosTCPCommunication
{
    ros::NodeHandle nh_;
    ros::Subscriber obj_sub_, ori_sub_;
    bool obj_received, ori_received;
    string obj;
    float ori;
    int sock = 0;
public:
    RosTCPCommunication();
    void objCb(const std_msgs::String& msg);
    void oriCb(const std_msgs::Float32& msg);
    void sendTCP();
};

#endif //ROB8_WS_ROSTCPCOMMUNICATION_H
