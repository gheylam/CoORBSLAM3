/**
 * This file is part of CoORBSLAM3
 *
 * Developer: TSZ HEY LAM
 * Year Developed: 2020-2021
 *
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "CoORBSLAM3/NewAgentFeed.h"
#include <cstdlib>

/*
 * agent_image is a ROS node that is run by external agents to participate
 * in the CoORBSLAM SLAM operation. Currently it takes a static set of images
 * from a directory and makes a ROS Service request for each image.
 */

int main(int argc, char **argv){

    ROS_INFO_STREAM("CV VERSION " << CV_MAJOR_VERSION);
  if(argc < 3){
      ROS_ERROR("Incorrect number of arguments provided for agent_image");
      std::cout << "agent_image <AgentId> <Path to Image Frames /*.png>" << std::endl;
      return 1;
  }
  cv::String sThisNode = "agent_" + cv::String(argv[1]);
  ros::init(argc, argv, sThisNode.c_str());

  cv::String sInitMsg = "Agent: " + sThisNode + " Initialized";
  ROS_INFO(sInitMsg.c_str());
  ros::NodeHandle pNodeHandle;
  ros::ServiceClient client = pNodeHandle.serviceClient<CoORBSLAM3::NewAgentFeed>("new_agent_feed");
  CoORBSLAM3::NewAgentFeed srv;

  //Load in all the images from the dataset
  std::vector<cv::String> vsFileNames;

  //TODO get filepath directory from commandline arugments [DONE]
  cv::String sArgPath = argv[2];
  int nLastCharIndex = sArgPath.length() - 1;
  if (sArgPath[nLastCharIndex] != '/'){
      sArgPath = sArgPath + "/*.png";
  }else{
      sArgPath = sArgPath + "*.png";
  }
  std::cout << sArgPath.c_str() << std::endl;
  cv::String sDatasetPath = sArgPath;
  std::cout << sDatasetPath << std::endl;

  std::cout << "Loading dataset, might take awhile..." << std::endl;
  cv::glob(sDatasetPath, vsFileNames, false);
  //Validate whether the path was correct
  if(vsFileNames.empty()){
      cv::String sBadPath = "No PNGs found at: " + sDatasetPath;
      ROS_ERROR(sBadPath.c_str());
      return 1;
  }
  std::cout << "Finished loading dataset" << std::endl;

  //create a cv to imgptr converter
  cv_bridge::CvImage pImgBridge;
  sensor_msgs::Image msgImage;
  cv::Mat mImage;

  //TODO Assign the AgentId through the commandline argument [DONE]
  srv.request.nAgentID = std::stoi(argv[1]);

  //Feed data at 20Hz (Could be adjusted)
  ros::Rate rate(20);

  int nImageNum = 0; //Index into the file names of each image
  while(ros::ok() && (nImageNum < vsFileNames.size())){
      std::cout << vsFileNames[nImageNum] << std::endl;
      mImage = cv::imread(vsFileNames[nImageNum], cv::IMREAD_UNCHANGED);
      if(mImage.empty()){
          ROS_ERROR_STREAM("Could not open or find the image at: " << vsFileNames[nImageNum]);
          ros::shutdown();
      }
      pImgBridge = cv_bridge::CvImage(std_msgs::Header(),
                                      sensor_msgs::image_encodings::MONO8, mImage);
      pImgBridge.toImageMsg(msgImage);

      srv.request.sImageMsg = msgImage;
      srv.request.header.stamp = ros::Time::now(); //Set the timestamp to the time this Image is sent

      if(client.call(srv)){
          ROS_INFO("Ack: %1d", (long int)srv.response.ack);
          nImageNum++;
      }else{
          ROS_ERROR("Failed to reach Server");
          //TODO Make the agent retry periodically
          usleep(1000000);
      }
      rate.sleep();
  }
}