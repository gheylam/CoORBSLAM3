/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<unistd.h>
#include<thread>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include<opencv2/core/core.hpp>
#include"../include/System.h"
#include"../include/ImgFrame.h"
#include "CoORBSLAM3/AddTwoInts.h"
#include "CoORBSLAM3/NewAgentFeed.h"

static const std::string OPENCV_WINDOW = "Image window";

bool add(CoORBSLAM3::AddTwoInts::Request &req,
         CoORBSLAM3::AddTwoInts::Response &res){
    res.sum = req.a + req.b;
    ROS_INFO("request: x=%1d, y=%1d", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%1d]", (long int)res.sum);
    return true;
}

bool feed(CoORBSLAM3::NewAgentFeed::Request &req,
          CoORBSLAM3::NewAgentFeed::Response &res){
    res.ack = 1;
    ROS_INFO("request: agentId=%1d", (long int)req.nAgentID);
    sensor_msgs::Image imageMsg = req.sImageMsg;
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImagePtr cv_ptr;
    cv::namedWindow(OPENCV_WINDOW);
    try{
        cv_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    return true;
}

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    bool GrabImageSrv(CoORBSLAM3::NewAgentFeed::Request &req,
                   CoORBSLAM3::NewAgentFeed::Response &res);

    void PassImage();

    ORB_SLAM3::System* mpSLAM;
private:
    std::list<ImgFrame*> mlNewImgFramesBuffer;
    std::mutex mMutexImgFrameListChanges;
};

int sentOnce = 0;
int startSystem = 0;
std::thread ptSystem;

int main(int argc, char **argv) {
    ros::init(argc, argv, "Mono_Agent");
    ros::start();

    if (argc != 3) {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    ros::NodeHandle nodeHandler;
    /*
    //Ensuring that subscribers are connected in the ROS network
    ros::Publisher clientPO_pub = nodeHandler.advertise<std_msgs::String>("OrbServer", 1000);
    ros::Rate poll_rate(100);
    while(clientPO_pub.getNumSubscribers() == 0){
        poll_rate.sleep();
    }
    */

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    ImageGrabber igb(&SLAM);

    if(startSystem == 0){
        ptSystem = thread(&ORB_SLAM3::System::Run, &SLAM);
        startSystem++;
        ROS_INFO("System started");
    }

    //starting up the rosservice
    ros::ServiceServer service = nodeHandler.advertiseService("new_agent_feed", &ImageGrabber::GrabImageSrv, &igb);
    ROS_INFO("Ready to grab images");
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}

bool ImageGrabber::GrabImageSrv(CoORBSLAM3::NewAgentFeed::Request &req,
                             CoORBSLAM3::NewAgentFeed::Response &res){
    res.ack = 1;
    //ROS_INFO("request: agentId=%1d", (long int)req.nAgentID);
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(req.sImageMsg);
        //cv_ptr = cv_bridge::toCvShare(msg);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }
    //std::cout << "header stamp: " << req.header.stamp.toSec() << std::endl;
    //mpSLAM->TrackMonocular(cv_ptr->image, req.header.stamp.toSec(), req.nAgentID);
    ImgFrame* pNewImgFrame = new ImgFrame(cv_ptr->image, req.header.stamp.toSec(), req.nAgentID);
    {
        unique_lock<mutex> lock(mMutexImgFrameListChanges);
        mlNewImgFramesBuffer.push_back(pNewImgFrame);
        std::cout << "Added new ImgFrame in ROS script" << std::endl;

    }

    //Pass any new frame to System
    PassImage();
    return true;
}

void ImageGrabber::PassImage() {
    unique_lock<mutex> lock(mMutexImgFrameListChanges);
    while(!mlNewImgFramesBuffer.empty()) {
        if (mpSLAM->GetAcceptingNewImgFrames()) {
            std::cout << "Passing image to System" << std::endl;
            ImgFrame *pCurrentImgFrame = mlNewImgFramesBuffer.front();
            mpSLAM->InsertImgFrame(pCurrentImgFrame);
            mlNewImgFramesBuffer.pop_front();
        }else{
            break;
        }
    }
}

