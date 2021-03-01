//
// Created by gheylam on 01/03/2021.
//

#include "../include/LoopClosingManager.h"

namespace ORB_SLAM3 {


    LoopClosingManager::LoopClosingManager(Atlas *pAtlas, KeyFrameDatabase *pDB, ORBVocabulary *pVoc,
                                           const bool bFixScale) {

    }

    void LoopClosingManager::Run() {
        //We ingest new KeyFrames sent from LocalMapper and
        //Pass them onto the correct LoopCloser object in our mmAgentToLoopCloser map

    }

    bool LoopClosingManager::CheckNewKeyFrames() {

    }

    void LoopClosingManager::ProcessNewKeyFrame() {

    }

    void LoopClosingManager::AddNewAgent(int nAgentId, Tracking* pNewTracker, Viewer* pViewer) {
        unique_lock <mutex> lock(mMutexNewAgent);

        //Check if this agent already had an entry in the map
        if (mmAgentToCloser.find(nAgentId) != mmAgentToCloser.end()) {
            std::cout << "LoopClosingManager::AddNewAgent() | Returning Agent" << std::endl;
            return;
        } else {
            std::cout << "LoopClosingManager::AddNewAgent() | New Agent, creating new LoopCloser" << std::endl;
            LoopClosing *pNewLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, mbFixScale);
            pNewLoopCloser->SetTracker(pNewTracker);
            pNewLoopCloser->SetLocalMapper(mpLocalMapper);
            pNewLoopCloser->mpViewer = pViewer;
            //pNewTracker->SetLoopClosing(pNewLoopCloser);

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



}