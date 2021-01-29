//
// Created by gheylam on 23/01/2021.
//

#include "ClientPO.h"
ClientPO::ClientPO(){
    client_pub = nh.advertise<std_msgs::String>("OrbServer", 1000);
    clientId = 0;
}

ClientPO::ClientPO(int id) {
    client_pub = nh.advertise<std_msgs::String>("OrbServer", 1000);
    clientId = id;
}

int ClientPO::getId() {
    return clientId;
}

void ClientPO::publish(const char* message){
    ROS_INFO("Publishing to server");
    std_msgs::String msg;
    msg.data = message;
    client_pub.publish(msg);
    ROS_INFO("%s", msg.data.c_str());
}
