//
// Created by gheylam on 01/03/2021.
//

#include "../include/LoopClosingManager.h"

namespace ORB_SLAM3 {


    LoopClosingManager::LoopClosingManager(Atlas *pAtlas, KeyFrameDatabase *pDB, ORBVocabulary *pVoc,
                                           const bool bFixScale): mbFinished(true), mbFinishRequested(false), mpAtlas(pAtlas),
                                           mpKeyFrameDatabase(pDB), mpVocabulary(pVoc){

    }

    void LoopClosingManager::Run() {
        //We ingest new KeyFrames sent from LocalMapper and
        //Pass them onto the correct LoopCloser object in our mmAgentToLoopCloser map
        mbFinished = false;

        while(1){
            if(CheckNewKeyFrames()){
                //Find the right LoopClosing object and run the TryLoopClose
                mpCurrentKeyFrame = mlpNewKeyFrames.front();
                mlpNewKeyFrames.pop_front();
                int nAgentId = mpCurrentKeyFrame->GetAgentId();
                mpCurrentLoopCloser = GetAgentLoopCloser(nAgentId);
                mpCurrentLoopCloser->TryLoopClose(mpCurrentKeyFrame);
            }

            //ResetIfRequested();

            if(CheckFinished()){
                std::cout << "LOOPCLOSINGMANAGER::Run() | Finish requested" << std::endl;
                break;
            }

            usleep(5000);

        }

        SetFinish();
    }

    void LoopClosingManager::InsertKeyFrame(KeyFrame* pKF){
        unique_lock<mutex> lock(mMutexNewKeyFrame);
        if(pKF->mnId!=0){
            mlpNewKeyFrames.push_back(pKF);
        }
    }

    bool LoopClosingManager::CheckNewKeyFrames() {
        unique_lock<mutex> lock(mMutexNewKeyFrame);
        if(!mlpNewKeyFrames.empty()){
            return true;
        }else{
            return false;
        }
    }

    void LoopClosingManager::AddNewAgent(int nAgentId, Tracking* pNewTracker, Viewer* pViewer) {
        unique_lock<mutex> lock(mMutexNewAgent);

        //Check if this agent already had an entry in the map
        if (mmAgentToCloser.find(nAgentId) != mmAgentToCloser.end()) {
            std::cout << "LoopClosingManager::AddNewAgent() | Returning Agent" << std::endl;
            return;
        } else {
            std::cout << "LoopClosingManager::AddNewAgent() | New Agent, creating new LoopCloser" << std::endl;
            LoopClosing *pNewLoopCloser = new LoopClosing(this, mpAtlas, mpKeyFrameDatabase, mpVocabulary, mbFixScale);
            pNewLoopCloser->SetTracker(pNewTracker);
            pNewLoopCloser->SetLocalMapper(mpLocalMapper);
            pNewLoopCloser->mpViewer = pViewer;
            pNewTracker->SetLoopClosing(pNewLoopCloser);

            //Add the new LoopCloser into the map
            mmAgentToCloser.insert(std::pair<int, LoopClosing *>(nAgentId, pNewLoopCloser));
            std::cout << "LoopClosingManager::AddNewAgent() | New LoopCloser for Agent: " << nAgentId
                      << " has been created" << std::endl;
        }
    }

    LoopClosing *LoopClosingManager::GetAgentLoopCloser(int nAgentId) {
        if (mmAgentToCloser.find(nAgentId) == mmAgentToCloser.end()) {
            std::cout << "LoopClosingManager::GetAgentLoopCloser() | ERROR, AgentId: " << nAgentId
                      << " does not have a LoopCloser" << std::endl;
            exit(1);
        } else {
            std::cout << "LoopClosingManager::GetAgentLoopCloser() | LoopCloser for AgentId: " << nAgentId << " found"
                      << std::endl;
            return mmAgentToCloser[nAgentId];
        }
    }

    void LoopClosingManager::SetLocalMapper(LocalMapping *pLocalMapper) {
        mpLocalMapper = pLocalMapper;
    }

    bool LoopClosingManager::CheckFinished(){
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void LoopClosingManager::RequestFinish(){
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    void LoopClosingManager::SetFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool LoopClosingManager::isFinished() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void LoopClosingManager::InformGBAStart(){
        unique_lock<mutex> lock(mMutexGBA);
        mbRunningGBA = true;
    }

    void LoopClosingManager::InformGBAEnd(){
        unique_lock<mutex> lock(mMutexGBA);
        mbRunningGBA = false;
    }

    bool LoopClosingManager::isRunningGBA(){
        unique_lock<mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }

}