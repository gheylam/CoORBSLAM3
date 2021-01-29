#include "ros/ros.h"
#include "std_msgs/String.h"

void agentCallback(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("I head: [%s]", msg->data.c_str());
}

int main(int argc, char**argv){
  ros::init(argc, argv, "server");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("OrbServer", 1000, agentCallback);

  ros::spin();

  return 0;
}
