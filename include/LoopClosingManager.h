//
// Created by gheylam on 01/03/2021.
//

#ifndef COORBSLAM3_LOOPCLOSINGMANAGER_H
#define COORBSLAM3_LOOPCLOSINGMANAGER_H

#include "KeyFrame.h"
#include "System.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Atlas.h"
#include "thread"
#include "mutex"

#include "ORBVocabulary.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"


/*
 * The LoopClosingManager takes in KeyFrames from LocalMapper and
 * distributes them to the different LoopCloser objects based on
 * the AgentId of the KeyFrame.
 *
 * Much like an ordinary LoopCloser object, it will constantly
 * monitor for new KeyFrames. It will also constantly monitor
 * when System adds a new Agent because this implies that a
 * a new LoopCloser object needs to be created.
 */

class LoopClosingManager {
public:
    LoopClosingManager(Atlas* pAtlas, KeyFrameDatabase* pDB, ORBVocabulary* pVoc, const bool bFixScale);
    void setLocalMapper();
    void Run(); //This is the function that System will fire a new thread on
    bool CheckNewKeyFrames(); // verifies whether there are any new KeyFrames in the queue
    void ProcessNewKeyFrame(); //Issues the correct LoopCloser object to deal with the new KeyFrame
    void AddNewAgent();
    void ProcessNewAgentQueue();
private:
    ORBVocabulary* mpVocabulary;
    KeyFrameDatabase* mpKeyFrameDatabase;
    Atlas* mpAtlas;
    LocalMapping* mpLocalMapper;

    std::mutex mMutexNewAgent;
    std::mutex mMutexNewKeyFrame;
    std::list<int> mvNewAgentIdQueue;
    std::list<KeyFrame*> mlNewKeyFrames;
    std::map<int, LoopClosing*> mmAgentToCloser;




};


#endif //COORBSLAM3_LOOPCLOSINGMANAGER_H
