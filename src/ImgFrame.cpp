//
// Created by gheylam on 05/02/2021.
//

#include "../include/ImgFrame.h"


ImgFrame::ImgFrame(cv::Mat &pImage, double dTimestamp, int nAgentId) {
    mmImage = pImage;
    mdTimestamp = dTimestamp;
    mnAgentId = nAgentId;
}

cv::Mat ImgFrame::GetImage() {
    return mmImage;
}

double ImgFrame::GetTimestamp() {
    return mdTimestamp;
}

int ImgFrame::GetAgentId() {
    return mnAgentId;
}
