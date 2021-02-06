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

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    //ROS service callback equivalent to existing subscriber callback.
    bool GrabImageSrv(CoORBSLAM3::NewAgentFeed::Request &req,
                   CoORBSLAM3::NewAgentFeed::Response &res);

    //Transfers buffered image frames over to System
    void PassImage();

    ORB_SLAM3::System* mpSLAM;
private:
    //list for buffering the incoming Image frames
    std::list<ImgFrame*> mlNewImgFramesBuffer;

    //mutex for ensuring thread safe changes to the Image Frame buffers
    std::mutex mMutexImgFrameListChanges;
};

int sentOnce = 0;
int startSystem = 0; //Evaluates whether the System thread has started
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

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    ImageGrabber igb(&SLAM);

    if(startSystem == 0){
        //Initialize System thread to ingest new Image Frames from this ROS code
        ptSystem = thread(&ORB_SLAM3::System::Run, &SLAM);
        startSystem++;
        ROS_INFO("System started");
    }

    //Initialize the ROS service
    ros::ServiceServer service = nodeHandler.advertiseService("new_agent_feed", &ImageGrabber::GrabImageSrv, &igb);
    ROS_INFO("Ready to grab images");

    //Removed subscriber for more confident ROS Service communication channel
    //ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

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
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(req.sImageMsg);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }
    ImgFrame* pNewImgFrame = new ImgFrame(cv_ptr->image, req.header.stamp.toSec(), req.nAgentID);
    {
        unique_lock<mutex> lock(mMutexImgFrameListChanges);
        mlNewImgFramesBuffer.push_back(pNewImgFrame);
        std::cout << "Added new ImgFrame in ROS script" << std::endl;

    }

    //Pass existing frames in the buffer to System thread
    PassImage();
    return true;
}

//Evaluates whether its safe to send transfer Image Frames to the
//System thread then send the Image Frames if it is.
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

