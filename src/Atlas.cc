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

#include "Atlas.h"
#include "Viewer.h"

#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"

namespace ORB_SLAM3
{

Atlas::Atlas(){
    mpCurrentMap = static_cast<Map*>(NULL);
}

Atlas::Atlas(int initKFid): mnLastInitKFidMap(initKFid), mHasViewer(false)
{
    mpCurrentMap = static_cast<Map*>(NULL);
    //CreateNewMap(); This is delayed until the first Image Frames come in
}

Atlas::~Atlas()
{
    for(std::set<Map*>::iterator it = mspMaps.begin(), end = mspMaps.end(); it != end;)
    {
        Map* pMi = *it;

        if(pMi)
        {
            delete pMi;
            pMi = static_cast<Map*>(NULL);

            it = mspMaps.erase(it);
        }
        else
            ++it;

    }
}

void Atlas::CreateNewMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Creation of new map with id: " << Map::nNextId << endl;
    if(mpCurrentMap){
        cout << "Exits current map " << endl;
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum

        mpCurrentMap->SetStoredMap();
        cout << "Saved map with ID: " << mpCurrentMap->GetId() << endl;

        //if(mHasViewer)
        //    mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
    }
    cout << "Creation of new map with last KF id: " << mnLastInitKFidMap << endl;

    mpCurrentMap = new Map(mnLastInitKFidMap);
    mpCurrentMap->SetCurrentMap();
    mspMaps.insert(mpCurrentMap);
}



void Atlas::CreateNewMap(int nAgentId)
{
    /*
     * Overloaded for CoORBSLAM
     * When we create a new map we need to create it with respect to
     * the agent that is creating it.
     */
    unique_lock<mutex> lock(mMutexAtlas);
    //std::lock_guard<std::recursive_mutex> lock(mRMutexAtlas);
    std::cout << "ATLAS | In Atlas::CreateNewMap(int nAgentId)" << std::endl;
    //First we will attempt to set mpCurrentMap to the AgentId
    if(mMapAgentMap.find(nAgentId) == mMapAgentMap.end()){
        std::cout << "ATLAS | No map exists for agent: " << nAgentId << " in CreateNewMap()" << std::endl;
        mpCurrentMap = static_cast<Map*>(NULL);
    }else{
        mpCurrentMap = mMapAgentMap.find(nAgentId)->second;
    }

    cout << "Creation of new map with id: " << Map::nNextId << endl;
    cout << "Map created for Agent: " << nAgentId << endl;
    if(mpCurrentMap){
        cout << "Exits current map " << endl;
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum

        mpCurrentMap->SetStoredMap();
        cout << "Saved map with ID: " << mpCurrentMap->GetId() << endl;

        //if(mHasViewer)
        //    mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
    }
    cout << "Creation of new map with last KF id: " << mnLastInitKFidMap << endl;
    mpCurrentMap = new Map(mnLastInitKFidMap);
    mpCurrentMap->SetCurrentMap();
    mspMaps.insert(mpCurrentMap);
    //Add the association between this new map and agentId
    if(mMapAgentMap.find(nAgentId) == mMapAgentMap.end()){
        cout << "New AgentID found" << endl;
        mMapAgentMap.insert(pair<int, Map*>(nAgentId, mpCurrentMap));
        //Also insert new accumulator for how many maps there are;
        mMapNumAgentMaps.insert(pair<int, int>(nAgentId, 1));
    }else{
        cout << "Existing AgentID found" << endl;
        mMapAgentMap[nAgentId] = mpCurrentMap;
        //Also update how many maps there are for each agentId
        mMapNumAgentMaps[nAgentId] += 1;
    }
}

void Atlas::ChangeMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Change to map with id: " << pMap->GetId() << endl;
    if(mpCurrentMap){
        mpCurrentMap->SetStoredMap();
    }

    mpCurrentMap = pMap;
    mpCurrentMap->SetCurrentMap();
}

//This mnLastInitKFidMAP is old and no longer used in the new algorithm
unsigned long int Atlas::GetLastInitKFid()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mnLastInitKFidMap;
}

void Atlas::SetViewer(Viewer* pViewer)
{
    mpViewer = pViewer;
    mHasViewer = true;
}

void Atlas::AddKeyFrame(KeyFrame* pKF)
{
    Map* pMapKF = pKF->GetMap();
    pMapKF->AddKeyFrame(pKF);
}

void Atlas::AddMapPoint(MapPoint* pMP)
{
    Map* pMapMP = pMP->GetMap();
    pMapMP->AddMapPoint(pMP);
}

void Atlas::AddCamera(GeometricCamera* pCam)
{
    mvpCameras.push_back(pCam);
}

void Atlas::SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs)
{
    unique_lock<mutex> lock(mMutexAtlas);


    mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs, int nAgentId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(mMapAgentMap.find(nAgentId) == mMapAgentMap.end()){
        std::cout << "ATLAS | ERROR attempted to access non-existent agent map" << std::endl;
        exit(1);
    }else{
        mpCurrentMap = mMapAgentMap.find(nAgentId)->second;
    }
    mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->InformNewBigChange();
}

void Atlas::InformNewBigChange(int nAgentId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(mMapAgentMap.find(nAgentId) == mMapAgentMap.end()){
        std::cout << "ATLAS | ERROR attempted to access non-existent agent map" << std::endl;
        exit(1);
    }else{
        mpCurrentMap = mMapAgentMap.find(nAgentId)->second;
    }
    mpCurrentMap->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetLastBigChangeIdx();
}

int Atlas::GetLastBigChangeIdx(int nAgentId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(mMapAgentMap.find(nAgentId) == mMapAgentMap.end()){
        std::cout << "ATLAS | ERROR attempted to access non-existent agent map" << std::endl;
        exit(1);
    }else{
        mpCurrentMap = mMapAgentMap.find(nAgentId)->second;
    }
    return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->MapPointsInMap();
}

long unsigned int Atlas::MapPointsInMap(int nAgentId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(mMapAgentMap.find(nAgentId) == mMapAgentMap.end()){
        std::cout << "ATLAS | ERROR attempted to access non-existent agent map" << std::endl;
        exit(1);
    }else{
        mpCurrentMap = mMapAgentMap.find(nAgentId)->second;
    }
    return mpCurrentMap->MapPointsInMap();
}

long unsigned Atlas::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->KeyFramesInMap();
}

long unsigned Atlas::KeyFramesInMap(int nAgentId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(mMapAgentMap.find(nAgentId) == mMapAgentMap.end()){
        std::cout << "ATLAS | ERROR attempted to access non-existent agent map" << std::endl;
        exit(1);
    }else{
        mpCurrentMap = mMapAgentMap.find(nAgentId)->second;
    }
    return mpCurrentMap->KeyFramesInMap();
}

std::vector<KeyFrame*> Atlas::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexAtlas);
    //std::cout << "Atlas::GetAllKeyFrames() | value of mpCurrentMap: " << mpCurrentMap << std::endl;
    return mpCurrentMap->GetAllKeyFrames();
}

std::vector<KeyFrame*> Atlas::GetAllKeyFrames(int nAgentId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(mMapAgentMap.find(nAgentId) == mMapAgentMap.end()){
        std::cout << "ATLAS | ERROR attempted to access non-existent agent map" << std::endl;
        exit(1);
    }else{
        mpCurrentMap = mMapAgentMap.find(nAgentId)->second;
    }
    return mpCurrentMap->GetAllKeyFrames();
}

std::vector<MapPoint*> Atlas::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMapPoints();
}

std::vector<MapPoint*> Atlas::GetAllMapPoints(int nAgentId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(mMapAgentMap.find(nAgentId) == mMapAgentMap.end()){
        std::cout << "ATLAS | ERROR attempted to access non-existent agent map" << std::endl;
        exit(1);
    }else{
        mpCurrentMap = mMapAgentMap.find(nAgentId)->second;
    }
    return mpCurrentMap->GetAllMapPoints();
}

std::vector<MapPoint*> Atlas::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetReferenceMapPoints();
}

std::vector<MapPoint*> Atlas::GetReferenceMapPoints(int nAgentId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(mMapAgentMap.find(nAgentId) == mMapAgentMap.end()){
        std::cout << "ATLAS | ERROR attempted to access non-existent agent map" << std::endl;
        exit(1);
    }else{
        mpCurrentMap = mMapAgentMap.find(nAgentId)->second;
    }
    return mpCurrentMap->GetReferenceMapPoints();
}

vector<Map*> Atlas::GetAllMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    struct compFunctor
    {
        inline bool operator()(Map* elem1 ,Map* elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    vector<Map*> vMaps(mspMaps.begin(),mspMaps.end());
    sort(vMaps.begin(), vMaps.end(), compFunctor());
    return vMaps;
}

long unsigned Atlas::GetAgentMapCount(int nAgentId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(mMapNumAgentMaps.find(nAgentId) == mMapNumAgentMaps.end()) {
        std::cout << "ATLAS: ERROR accessing non existent AgentId in GetAgentMapCount" << std::endl;
        exit(1);
    }else{
        int nAgentMapCount = mMapNumAgentMaps.find(nAgentId)->second;
        return  nAgentMapCount;
    }
}

int Atlas::CountMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mspMaps.size();
}

void Atlas::clearMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->clear();
}

void Atlas::clearMap(int nAgentId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(mMapAgentMap.find(nAgentId) == mMapAgentMap.end()){
        std::cout << "ATLAS | ERROR attempted to access non-existent agent map" << std::endl;
        exit(1);
    }else{
        mpCurrentMap = mMapAgentMap.find(nAgentId)->second;
    }
    mpCurrentMap->clear();
}

void Atlas::clearAtlas()
{
    unique_lock<mutex> lock(mMutexAtlas);
    /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
    {
        (*it)->clear();
        delete *it;
    }*/
    mspMaps.clear();
    for(int nAgentMapEntry = 0; nAgentMapEntry < mMapAgentMap.size(); nAgentMapEntry++){
        mMapAgentMap[nAgentMapEntry] = static_cast<Map*>(NULL);
    }
    mpCurrentMap = static_cast<Map*>(NULL);
    mnLastInitKFidMap = 0;
}

Map* Atlas::GetCurrentMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(!mpCurrentMap)
        CreateNewMap();
    while(mpCurrentMap->IsBad())
        usleep(3000);
    return mpCurrentMap;
}

Map* Atlas::GetCurrentMap(int nAgentId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    //std::lock_guard<std::recursive_mutex> lock(mRMutexAtlas);
    //Get the CurrentMap pointer that corresponds with the given AgentId
    std::cout << "ATLAS | In Atlas::GetCurrentMap(int nAgentId)" << std::endl;
    if(mMapAgentMap.find(nAgentId) == mMapAgentMap.end()){
        std::cout << "ATLAS | CurrentMap for ID not found, creating new map" << std::endl;
        std::cout << "ATLAS | In Atlas::GetCurrentMap(int nAgentId) Entering CreateNewMap()" << std::endl;
        CreateNewMap(nAgentId);
        std::cout << "ATLAS | In Atlas::GetCurrentMap(int nAgentId) Finished creating a new map" << std::endl;
    }else{
        std::cout << "ATLAS | CurrentMap found for AgentId: " << nAgentId << std::endl;
        mpCurrentMap = mMapAgentMap.find(nAgentId)->second;
    }
    //if(!mpCurrentMap)
    //    std::cout << "ATLAS | In Atlas::GetCurrentMap(int nAgentId) Entering CreateNewMap()" << std::endl;
    //    CreateNewMap(nAgentId);
    //    std::cout << "ATLAS | In Atlas::GetCurrentMap(int nAgentId) Finished creating a new map" << std::endl;
    while(mpCurrentMap->IsBad())
        usleep(3000);
    return mpCurrentMap;
}

void Atlas::SetMapBad(Map* pMap)
{
    mspMaps.erase(pMap);
    pMap->SetBad();

    mspBadMaps.insert(pMap);
}

void Atlas::RemoveBadMaps()
{
    /*for(Map* pMap : mspBadMaps)
    {
        delete pMap;
        pMap = static_cast<Map*>(NULL);
    }*/
    mspBadMaps.clear();
}

bool Atlas::isInertial()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->IsInertial();
}

bool Atlas::isInertial(int nAgentId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(mMapAgentMap.find(nAgentId) == mMapAgentMap.end()){
        std::cout << "ATLAS | ERROR attempted to access non-existent agent map" << std::endl;
        exit(1);
    }else{
        mpCurrentMap = mMapAgentMap.find(nAgentId)->second;
    }
    return mpCurrentMap->IsInertial();
}

void Atlas::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetInertialSensor();
}

void Atlas::SetInertialSensor(int nAgentId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(mMapAgentMap.find(nAgentId) == mMapAgentMap.end()){
        std::cout << "ATLAS | ERROR attempted to access non-existent agent map" << std::endl;
        exit(1);
    }else{
        mpCurrentMap = mMapAgentMap.find(nAgentId)->second;
    }
    mpCurrentMap->SetInertialSensor();
}

void Atlas::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetImuInitialized();
}

void Atlas::SetImuInitialized(int nAgentId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(mMapAgentMap.find(nAgentId) == mMapAgentMap.end()){
        std::cout << "ATLAS | ERROR attempted to access non-existent agent map" << std::endl;
        exit(1);
    }else{
        mpCurrentMap = mMapAgentMap.find(nAgentId)->second;
    }
    mpCurrentMap->SetImuInitialized();
}

bool Atlas::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->isImuInitialized();
}

bool Atlas::isImuInitialized(int nAgentId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(mMapAgentMap.find(nAgentId) == mMapAgentMap.end()){
        std::cout << "ATLAS | ERROR attempted to access non-existent agent map" << std::endl;
        exit(1);
    }else{
        mpCurrentMap = mMapAgentMap.find(nAgentId)->second;
    }
    return mpCurrentMap->isImuInitialized();
}

void Atlas::PreSave()
{
    if(mpCurrentMap){
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum
    }

    struct compFunctor
    {
        inline bool operator()(Map* elem1 ,Map* elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    std::copy(mspMaps.begin(), mspMaps.end(), std::back_inserter(mvpBackupMaps));
    sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());

    std::set<GeometricCamera*> spCams(mvpCameras.begin(), mvpCameras.end());
    cout << "There are " << spCams.size() << " cameras in the atlas" << endl;
    for(Map* pMi : mvpBackupMaps)
    {
        cout << "Pre-save of map " << pMi->GetId() << endl;
        pMi->PreSave(spCams);
    }
    cout << "Maps stored" << endl;
    for(GeometricCamera* pCam : mvpCameras)
    {
        cout << "Pre-save of camera " << pCam->GetId() << endl;
        if(pCam->GetType() == pCam->CAM_PINHOLE)
        {
            mvpBackupCamPin.push_back((Pinhole*) pCam);
        }
        else if(pCam->GetType() == pCam->CAM_FISHEYE)
        {
            mvpBackupCamKan.push_back((KannalaBrandt8*) pCam);
        }
    }

}

void Atlas::PostLoad()
{
    mvpCameras.clear();
    map<unsigned int,GeometricCamera*> mpCams;
    for(Pinhole* pCam : mvpBackupCamPin)
    {
        //mvpCameras.push_back((GeometricCamera*)pCam);
        mvpCameras.push_back(pCam);
        mpCams[pCam->GetId()] = pCam;
    }
    for(KannalaBrandt8* pCam : mvpBackupCamKan)
    {
        //mvpCameras.push_back((GeometricCamera*)pCam);
        mvpCameras.push_back(pCam);
        mpCams[pCam->GetId()] = pCam;
    }

    mspMaps.clear();
    unsigned long int numKF = 0, numMP = 0;
    map<long unsigned int, KeyFrame*> mpAllKeyFrameId;
    for(Map* pMi : mvpBackupMaps)
    {
        cout << "Map id:" << pMi->GetId() << endl;
        mspMaps.insert(pMi);
        map<long unsigned int, KeyFrame*> mpKeyFrameId;
        pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpKeyFrameId, mpCams);
        mpAllKeyFrameId.insert(mpKeyFrameId.begin(), mpKeyFrameId.end());
        numKF += pMi->GetAllKeyFrames().size();
        numMP += pMi->GetAllMapPoints().size();
    }

    cout << "Number KF:" << numKF << "; number MP:" << numMP << endl;
    mvpBackupMaps.clear();
}

void Atlas::SetKeyFrameDababase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

KeyFrameDatabase* Atlas::GetKeyFrameDatabase()
{
    return mpKeyFrameDB;
}

void Atlas::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBVocabulary = pORBVoc;
}

ORBVocabulary* Atlas::GetORBVocabulary()
{
    return mpORBVocabulary;
}

long unsigned int Atlas::GetNumLivedKF()
{
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for(Map* mMAPi : mspMaps)
    {
        num += mMAPi->GetAllKeyFrames().size();
    }

    return num;
}

long unsigned int Atlas::GetNumLivedMP() {
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for (Map *mMAPi : mspMaps) {
        num += mMAPi->GetAllMapPoints().size();
    }

    return num;
}

int Atlas::GetNumAgentContext(){
    unique_lock<mutex> lock(mMutexAtlas);
    return mMapAgentMap.size();
}

} //namespace ORB_SLAM3
