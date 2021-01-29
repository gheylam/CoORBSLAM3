//
// Created by gheylam on 18/01/2021.
//

#ifndef ORB_SLAM3_CLIENT_H
#define ORB_SLAM3_CLIENT_H
#include "ros/ros.h"
#include "std_msgs/String.h"

class OrbSlamClient{
public:
    OrbSlamClient();
    OrbSlamClient(int id);
    int getId();
    void publish();
private:
    int clientId;
    ros::NodeHandle nh;
    ros::Publisher client_pub;
};
#endif //ORB_SLAM3_CLIENT_H
