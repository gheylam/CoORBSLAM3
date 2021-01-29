//
// Created by gheylam on 23/01/2021.
//

#ifndef ORB_SLAM3_CLIENTPO_H
#define ORB_SLAM3_CLIENTPO_H
#include "ros/ros.h"
#include "std_msgs/String.h"

class ClientPO {
public:
    ClientPO();
    ClientPO(int id);
    int getId();
    void publish(const char*);
private:
    int clientId;
    ros::NodeHandle nh;
    ros::Publisher client_pub;
};


#endif //ORB_SLAM3_CLIENTPO_H
