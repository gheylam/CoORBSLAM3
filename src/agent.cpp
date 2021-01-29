#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv){
  ros::init(argc, argv, "agent");
  ros::NodeHandle nh;
  ros::Publisher agent_pub = nh.advertise<std_msgs::String>("OrbServer", 1000);

  //A ros::Rate object allows specification of loop frequency in Hz.
  ros::Rate loop_rate(10);
  
  int count = 0;
  while (ros::ok()){
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world" << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    agent_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
}
    
