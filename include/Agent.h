//
// Created by gheylam on 07/02/2021.
//

#ifndef COORBSLAM3_AGENT_H
#define COORBSLAM3_AGENT_H

#include<opencv2/core/core.hpp>
#include<iostream>

class Agent {
public:
    Agent(int nAgentId, double fCreatedTimestamp);
    void SetCameraParams(cv::String sCameraType, double fFx, double fFy, double fCx, double fCy,
                         double fK1, double fK2, double fP1, double fP2, int nCameraWidth,
                         int nCameraHeight, double fFPS, int nRGB, double fK3 = 0);

    void SetORBExtractorParams(int nFeatures, double fScaleFactor, int nLevels,
                         int nIniThFAST, int nMinThFAST);

    void SetViewerParams(double fKeyFrameSize, int nKeyFrameLineWidth, double fGraphLineWidth,
                         int nPointSize, double fCameraSize, int nCameraLineWidth,
                         double fViewpointX, double fViewpointY, double fViewpointZ,
                         double fViewpointF);

    int GetId();

private:
    double mfLastJoined;
    int mnAgentId;
    cv::String msCameraType;

    //Camera intrinsics
    double mfFx;
    double mfFy;
    double mfCx;
    double mfCy;

    double mfK1;
    double mfK2;
    double mfK3;
    double mfP1;
    double mfP2;

    int mnCameraWidth;
    int mnCameraHeight;

    double mfFPS;
    bool mbRGB;

    //ORB Extractor parameters
    int mnFeatures;
    double mfScaleFactor;
    int mnLevels;

    int mnIniThFAST;
    int mnMinThFAST;

    //Viewer parameters
    double mfKeyFrameSize;
    int mnKeyFrameLineWidth;
    double mfGraphLineWidth;
    int mnPointSize;
    double mfCameraSize;
    int mnCameraLineWidth;
    double mfViewpointX;
    double mfViewpointY;
    double mfViewpointZ;
    double mfViewpointF;
};

#endif //COORBSLAM3_AGENT_H
