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
#include "CoORBSLAM3/NewAgentRequest.h"
#include <cstdlib>

/*
 * agent_image is a ROS node that is run by external agents to participate
 * in the CoORBSLAM SLAM operation. Currently it takes a static set of images
 * from a directory and makes a ROS Service request for each image.
 */

bool ParseParamFile(CoORBSLAM3::NewAgentRequest& srv, cv::String sPath){
    cv::FileStorage fSettings(sPath, cv::FileStorage::READ);

    cv::FileNode openedFileTest = fSettings.getFirstTopLevelNode();
    if(openedFileTest.empty()){
       ROS_ERROR((cv::String("Could not read param file at: ") + sPath).c_str());
       return false;
    }

    cv::String sCameraName = fSettings["Camera.type"];
    if(!sCameraName.empty()) {
        srv.request.sCameraType = fSettings["Camera.type"].string();
    }else{
        ROS_ERROR("Camera.type parameter doesn't exist");
        return false;
    }

    //Parsing camera intrinsics
    cv::FileNode node = fSettings["Camera.fx"];
    if(!node.empty() && node.isReal()){
        srv.request.fx = node.real();
    }else{
        ROS_ERROR("Camera.fx parameter doesn't exist or is not a real number");
        return false;
    }

    node = fSettings["Camera.fy"];
    if(!node.empty() && node.isReal()){
        srv.request.fy = node.real();
    }else{
        ROS_ERROR("Camera.fy parameter doesn't exist or is not a real number ");
        return false;
    }

    node = fSettings["Camera.cx"];
    if(!node.empty() && node.isReal()){
        srv.request.cx = node.real();
    }else{
        ROS_ERROR("Camera.cx parameter doesn't exist or is not a real number");
        return false;
    }

    node = fSettings["Camera.cy"];
    if(!node.empty() && node.isReal()){
        srv.request.cy = node.real();
    }else{
        ROS_ERROR("Camera.cy parameter doesn't exist or is not a real number");
        return false;
    }

    //Parsing distortion parameters
    node = fSettings["Camera.k1"];
    if(!node.empty() && node.isReal()){
        srv.request.k1 = node.real();
    }else{
        ROS_ERROR("Camera.k1 parameter doesn't exist or is not a real number");
        return false;
    }

    node = fSettings["Camera.k2"];
    if(!node.empty() && node.isReal()){
        srv.request.k2 = node.real();
    }else{
        ROS_ERROR("Camera.k2 parameter doesn't exist or is not a real number");
        return false;
    }

    node = fSettings["Camera.p1"];
    if(!node.empty() && node.isReal()){
        srv.request.p1 = node.real();
    }else{
        ROS_ERROR("Camera.p1 parameter doesn't exist or is not a real number");
        return false;
    }

    node = fSettings["Camera.p2"];
    if(!node.empty() && node.isReal()){
        srv.request.p2 = node.real();
    }else{
        ROS_ERROR("Camera.p2 parameter doesn't exist or is not a real number");
        return false;
    }

    node = fSettings["Camera.k3"];
    if(!node.empty() && node.isReal()){
        srv.request.k3 = node.real();
    }

    node = fSettings["Camera.width"];
    if(!node.empty() && node.isInt()){
        srv.request.nCameraWidth = node.operator int();
    }else{
        ROS_ERROR("Camera.width parameter doesn't exist or is not an integer");
        return false;
    }

    node = fSettings["Camera.height"];
    if(!node.empty() && node.isInt()){
        srv.request.nCameraHeight = node.operator int();
    }else{
        ROS_ERROR("Camera.height parameter doesn't exist or is not an integer");
        return false;
    }

    //FPS and Color parameters
    node = fSettings["Camera.fps"];
    if(!node.empty() && node.isReal()){
        srv.request.fps = node.real();
    }else{
        ROS_ERROR("Camera.fps parameter doesn't exist or is not a real number so defaulted to 30");
        srv.request.fps = 30.0;
    }

    node = fSettings["Camera.RGB"];
    if(!node.empty() && node.isInt()){
        srv.request.RGB = node.operator int();
    }else{
        ROS_ERROR("Camera.RGB parameter doesn't exist or is not a integer");
        return false;
    }

    //Distortion parameters
    node = fSettings["ORBextractor.nFeatures"];
    if(!node.empty() && node.isInt()){
        srv.request.nFeatures = node.operator int();
    }else{
        ROS_ERROR("ORBextractor.nFeatures parameter doesn't exist or is not an integer");
        return false;
    }

    node = fSettings["ORBextractor.scaleFactor"];
    if(!node.empty() && node.isReal()){
        srv.request.dScaleFactor = node.real();
    }else{
        ROS_ERROR("ORBextractor.scaleFactor parameter doesn't exist or is not a real number");
        return false;
    }

    node = fSettings["ORBextractor.nLevels"];
    if(!node.empty() && node.isInt()){
        srv.request.nLevels = node.operator int();
    }else{
        ROS_ERROR("ORBextractor.nLevels parameter doesn't exist or is not an integer");
        return false;
    }

    node = fSettings["ORBextractor.iniThFAST"];
    if(!node.empty() && node.isInt()){
        srv.request.nIniThFAST = node.operator int();
    }else{
        ROS_ERROR("ORBextractor.iniThFAST parameter doesn't exist or is not an integer");
        return false;
    }

    node = fSettings["ORBextractor.minThFAST"];
    if(!node.empty() && node.isInt()){
        srv.request.nMinThFAST = node.operator int();
    }else{
        ROS_ERROR("ORBextractor.minThFAST parameter doesn't exist or is not an integer");
        return false;
    }

    //Viewer parameters
    node = fSettings["Viewer.KeyFrameSize"];
    if(!node.empty() && node.isReal()){
        srv.request.dKeyFrameSize = node.real();
    }else{
        ROS_ERROR("Viewer.KeyFrameSize parameter doesn't exist or is not a real number");
        return false;
    }

    node = fSettings["Viewer.KeyFrameLineWidth"];
    if(!node.empty() && node.isInt()){
        srv.request.nKeyFrameLineWidth = node.operator int();
    }else{
        ROS_ERROR("Viewer.KeyFrameLineWidth parameter doesn't exist or is not an integer");
        return false;
    }

    node = fSettings["Viewer.GraphLineWidth"];
    if(!node.empty() && node.isReal()){
        srv.request.dGraphLineWidth = node.real();
    }else{
        ROS_ERROR("Viewer.GraphLineWidth parameter doesn't exist or is not a real number");
        return false;
    }

    node = fSettings["Viewer.PointSize"];
    if(!node.empty() && node.isInt()){
        srv.request.nPointSize = node.operator int();
    }else{
        ROS_ERROR("Viewer.PointSize parameter doesn't exist or is not an integer");
        return false;
    }

    node = fSettings["Viewer.CameraSize"];
    if(!node.empty() && node.isReal()){
        srv.request.dCameraSize = node.real();
    }else{
        ROS_ERROR("Viewer.CameraSize parameter doesn't exist or is not a real number");
        return false;
    }

    node = fSettings["Viewer.CameraLineWidth"];
    if(!node.empty() && node.isInt()){
        srv.request.nCameraLineWidth = node.operator int();
    }else{
        ROS_ERROR("Viewer.CameraLineWidth parameter doesn't exist or is not an integer");
        return false;
    }

    node = fSettings["Viewer.ViewpointX"];
    if(!node.empty() && (node.isReal() || node.isInt())){
        srv.request.dViewpointX = node.real();
    }else{
        ROS_ERROR("Viewer.ViewportX parameter doesn't exist or is not a real number");
        return false;
    }

    node = fSettings["Viewer.ViewpointY"];
    if(!node.empty() && (node.isReal() || node.isInt())){
        srv.request.dViewpointY = node.real();
    }else{
        ROS_ERROR("Viewer.ViewportY parameter doesn't exist or is not a real number");
        return false;
    }


    node = fSettings["Viewer.ViewpointZ"];
    if(!node.empty() && (node.isReal() || node.isInt())){
        srv.request.dViewpointZ = node.real();
    }else{
        ROS_ERROR("Viewer.ViewportZ parameter doesn't exist or is not a real number");
        return false;
    }


    node = fSettings["Viewer.ViewpointF"];
    if(!node.empty() && (node.isReal() || node.isInt())){
        srv.request.dViewpointF = node.real();
    }else{
        ROS_ERROR("Viewer.ViewportF parameter doesn't exist or is not a real number");
        return false;
    }

    return true;
}

int main(int argc, char **argv){

    ROS_INFO_STREAM("CV VERSION " << CV_MAJOR_VERSION);
  if(argc < 4){
      ROS_ERROR("Incorrect number of arguments provided for agent_image");
      std::cout << "agent_image <AgentId> <Path to Image Frames> <Path to Param file>" << std::endl;
      return 1;
  }

  //Check if agentID is an integer
  int nAgentId;
  try{
      nAgentId = std::stoi(argv[1]);
  }catch(...){
      ROS_ERROR("AgentId passed is not an integer, please use an integer");
      return 1;
  }

  cv::String sThisNode = "agent_" + cv::String(argv[1]);
  ros::init(argc, argv, sThisNode.c_str());

  cv::String sInitMsg = "Agent: " + sThisNode + " Initialized";
  ROS_INFO(sInitMsg.c_str());
  ros::NodeHandle pNodeHandle;
  //Service for requesting server to add this agent into the SLAM operation
  ros::ServiceClient clientNewAgent = pNodeHandle.serviceClient<CoORBSLAM3::NewAgentRequest>("new_agent_request");
  CoORBSLAM3::NewAgentRequest srvNewAgentReq;
  bool bParsed =  ParseParamFile(srvNewAgentReq, argv[3]);
  if(!bParsed){
      ROS_ERROR("Failed to parse parameter file");
      return 1;
  }
  //Send a request to server to add this agent as part of the SLAM operation
  srvNewAgentReq.request.header.stamp = ros::Time::now();
  srvNewAgentReq.request.nAgentId = nAgentId;

  if(clientNewAgent.call(srvNewAgentReq)){
      bool bSuccess = (bool)srvNewAgentReq.response.bCreated;
      if(bSuccess){
          ROS_INFO("Agent has joined the SLAM operation.");
      }else{
          ROS_INFO("Agent has failed to join the SLAM operation.");
          return 1;
      }
  }else{
        ROS_ERROR("Failed to reach Server, shutting down");
        return 1;
  }

  //Service for passing new image frames to the server
  ros::ServiceClient clientNewImg = pNodeHandle.serviceClient<CoORBSLAM3::NewAgentFeed>("new_agent_feed");
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
  try {
      cv::glob(sDatasetPath, vsFileNames, false);
  }catch(...){
      ROS_ERROR((cv::String("Could not read directory at: ") + sDatasetPath).c_str());
      return 1;
  }
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

      if(clientNewImg.call(srv)){
          ROS_INFO("Ack: %1d", (long int)srv.response.ack);
          nImageNum++;
      }else{
          ROS_ERROR("Failed to reach Server, trying again in 5 seconds");
          //TODO Make the agent retry periodically [DONE]
          usleep(5000000);
      }
      rate.sleep();
  }
}