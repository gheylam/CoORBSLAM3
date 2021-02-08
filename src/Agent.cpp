//
// Created by gheylam on 07/02/2021.
//

#include "../include/Agent.h"

Agent::Agent(int nAgentId, double fCreatedTimestamp) {
    mnAgentId = nAgentId;
    mfLastJoined = fCreatedTimestamp;
}

void Agent::SetCameraParams(cv::String sCameraType, double fFx, double fFy, double fCx, double fCy,
                            double fK1, double fK2, double fP1, double fP2, int nCameraWidth,
                            int nCameraHeight, double fFPS, int nRGB, double fK3) {
    msCameraType = sCameraType;
    mfFx = fFx;
    mfFy = fFy;
    mfCx = fCx;
    mfCy = fCy;

    mfK1 = fK1;
    mfK2 = fK2;
    mfK3 = fK3;
    mfP1 = fP1;
    mfP2 = fP2;

    mnCameraWidth = nCameraWidth;
    mnCameraHeight = nCameraHeight;

    mfFPS = fFPS;
    mbRGB = (bool)nRGB;
}

void Agent::SetORBExtractorParams(int nFeatures, double fScaleFactor, int nLevels, int nIniThFAST, int nMinThFAST) {
    mnFeatures = nFeatures;
    mfScaleFactor = fScaleFactor;
    mnLevels = nLevels;
    mnIniThFAST = nIniThFAST;
    mnMinThFAST = nMinThFAST;
}

void Agent::SetViewerParams(double fKeyFrameSize, int nKeyFrameLineWidth, double fGraphLineWidth, int nPointSize,
                            double fCameraSize, int nCameraLineWidth, double fViewpointX, double fViewpointY,
                            double fViewpointZ, double fViewpointF) {
    mfKeyFrameSize = fKeyFrameSize;
    mnKeyFrameLineWidth = nKeyFrameLineWidth;
    mfGraphLineWidth = fGraphLineWidth;
    mnPointSize = nPointSize;
    mfCameraSize = fCameraSize;
    mnCameraLineWidth = nCameraLineWidth;
    mfViewpointX = fViewpointX;
    mfViewpointY = fViewpointY;
    mfViewpointZ = fViewpointZ;
    mfViewpointF = fViewpointF;

}

int Agent::GetId() {
    return mnAgentId;
}