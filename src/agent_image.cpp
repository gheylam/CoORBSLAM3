#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui.hpp> 

int main(int argc, char **argv){
  ros::init(argc, argv, "agent");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("CV VERSION " << CV_MAJOR_VERSION);
  
  //create an image publisher
  image_transport::ImageTransport it(nh); 
  image_transport::Publisher image_pub = it.advertise("/camera/image_raw", 1);

  //create a cv to imgptr converter
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;
  
  cv::String imageName("/home/gheylam/catkin_ws/src/collaborative_orbslam3/src/orangepeel.png");
  cv::Mat image;
  image = cv::imread(imageName, cv::IMREAD_COLOR);
  if(image.empty()){
    ROS_ERROR_STREAM("Could not open or find the image at: " << imageName);
    ros::shutdown();
  }
  
  img_bridge = cv_bridge::CvImage(std_msgs::Header(),
				  sensor_msgs::image_encodings::RGB8, image);
  img_bridge.toImageMsg(img_msg); 
    
  ros::Rate rate(1);
  while(ros::ok()){
    image_pub.publish(img_msg);
    rate.sleep();
  }
}
    
