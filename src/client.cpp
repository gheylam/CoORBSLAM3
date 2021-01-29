//
// Created by gheylam on 18/01/2021.
//

#include "client.h"
OrbSlamClient::OrbSlamClient(){
    client_pub = nh.advertise<std_msgs::String>("OrbServer", 1000);
    clientId = 0;
}

OrbSlamClient::OrbSlamClient(int id) {
    client_pub = nh.advertise<std_msgs::String>("OrbServer", 1000);
    clientId = id;
}

int OrbSlamClient::getId() {
    return clientId;
}

void OrbSlamClient::publish() {
    ros::Rate loop_rate(10);
    ROS_INFO("Publishing to server");
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Hello Server" << std::endl;
    msg.data = ss.str();
    client_pub.publish(msg);
    ROS_INFO("%s", msg.data.c_str());

}
