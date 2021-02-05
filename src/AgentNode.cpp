#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "CoORBSLAM3/NewAgentFeed.h"
#include <cstdlib>

//static const std::string OPENCV_WINDOW = "Image window";

int main(int argc, char **argv){
    ros::init(argc, argv, "agent");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<CoORBSLAM3::NewAgentFeed>("new_agent_feed");
    CoORBSLAM3::NewAgentFeed srv;

    ROS_INFO_STREAM("CV VERSION " << CV_MAJOR_VERSION);
    //create an image publisher UPDATE 1/02/2021 we are no longer using publishers
    //UPDATE 02/02/2021 we need to use publishers because service is far too slow
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("/camera/image_raw", 1);


    //Load in all the images from the dataset
    std::cout << "Loading dataset, might take awhile..." << std::endl;
    std::vector<cv::String> vsFileNames;
    cv::String sDatasetPath = "/home/gheylam/orb_slam/datasets/MH01/mav0/cam0/data/*.png";
    cv::glob(sDatasetPath, vsFileNames, false);
    std::cout << "Finished loading dataset" << std::endl;

    //create a cv to imgptr converter
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image msgImage;
    cv::Mat mImage;

    //cv::namedWindow(OPENCV_WINDOW);

    srv.request.nAgentID = 1997;
    ros::Rate rate(10);
    int nImageNum = 0;
    while(ros::ok() && (nImageNum < vsFileNames.size())){
        mImage = cv::imread(vsFileNames[nImageNum], cv::IMREAD_UNCHANGED);
        if(mImage.empty()){
            ROS_ERROR_STREAM("Could not open or find the image at: " << vsFileNames[nImageNum]);
            ros::shutdown();
        }
        //cv::imshow(OPENCV_WINDOW, mImage);
        img_bridge = cv_bridge::CvImage(std_msgs::Header(),
                                        sensor_msgs::image_encodings::MONO8, mImage);
        img_bridge.toImageMsg(msgImage);

        srv.request.sImageMsg = msgImage;

        image_pub.publish(msgImage);
        /*
        if(client.call(srv)){
            ROS_INFO("Ack: %1d", (long int)srv.response.ack);
        }else{
            ROS_ERROR("Failed to call service new_agent_feed");
            return 1;
        }
        */
        rate.sleep();
        nImageNum++;
    }
}

