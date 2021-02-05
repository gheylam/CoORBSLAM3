//
// Created by gheylam on 05/02/2021.
//

#ifndef COORBSLAM3_IMGFRAME_H
#define COORBSLAM3_IMGFRAME_H

#include<opencv2/core/core.hpp>
#include<iostream>

class ImgFrame {
public:
    ImgFrame(cv::Mat &pImage, double dTimestamp, int nAgentId);

    cv::Mat GetImage();

    double GetTimestamp();

    int GetAgentId();

private:
    cv::Mat mmImage;
    double mdTimestamp;
    int mnAgentId;
};

#endif //COORBSLAM3_IMGFRAME_H
