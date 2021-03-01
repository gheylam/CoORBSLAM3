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

#ifndef ATLAS_H
#define ATLAS_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"
#include "Agent.h"

#include <set>
#include <mutex>
#include <map>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>


namespace ORB_SLAM3
{
class Viewer;
class Map;
class MapPoint;
class KeyFrame;
class KeyFrameDatabase;
class Frame;
class KannalaBrandt8;
class Pinhole;

//BOOST_CLASS_EXPORT_GUID(Pinhole, "Pinhole")
//BOOST_CLASS_EXPORT_GUID(KannalaBrandt8, "KannalaBrandt8")

class Atlas
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        //ar.template register_type<Pinhole>();
        //ar.template register_type<KannalaBrandt8>();

        // Save/load the set of maps, the set is broken in libboost 1.58 for ubuntu 16.04
        //ar & mspMaps;
        ar & mvpBackupMaps;
        ar & mvpCameras;
        //ar & mvpBackupCamPin;
        //ar & mvpBackupCamKan;
        // Need to save/load the static Id from Frame, KeyFrame, MapPoint and Map
        ar & Map::nNextId;
        ar & Frame::nNextId;
        ar & KeyFrame::nNextId;
        ar & MapPoint::nNextId;
        ar & GeometricCamera::nNextId;
        ar & mnLastInitKFidMap;
    }

public:
    Atlas();
    Atlas(int initKFid); // When its initialization the first map is created
    ~Atlas();
    void CreateNewMap();
    void CreateNewMap(int nAgentId);

    void ChangeMap(Map* pMap);

    void ChangeMap(Map* pMap, int nAgentId);

    unsigned long int GetLastInitKFid();

    void SetViewer(Viewer* pViewer);

    // Method for change components in the current map
    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    //void EraseMapPoint(MapPoint* pMP);
    //void EraseKeyFrame(KeyFrame* pKF);

    void AddCamera(GeometricCamera* pCam);

    /* All methods without Map pointer work on current map */
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs, int nAgentId);
    void InformNewBigChange();
    void InformNewBigChange(int nAgentId);
    int GetLastBigChangeIdx();
    int GetLastBigChangeIdx(int nAgentId);

    long unsigned int MapPointsInMap();
    long unsigned int MapPointsInMap(int nAgentId);
    long unsigned KeyFramesInMap();
    long unsigned KeyFramesInMap(int nAgentId);

    // Method for get data in current map
    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<KeyFrame*> GetAllKeyFrames(int nAgentId);

    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetAllMapPoints(int nAgentId);

    std::vector<MapPoint*> GetReferenceMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints(int nAgentId);

    vector<Map*> GetAllMaps();
    vector<Map*> GetAllAgentMaps(int nAgentId);
    long unsigned GetAgentMapCount(int nAgentId);

    int CountMaps();

    void clearMap();
    void clearMap(int nAgentId);

    void clearAtlas();

    Map* GetCurrentMap();
    Map* GetCurrentMap(int nAgentId);

    void SetMapBad(Map* pMap);
    void RemoveBadMaps();

    bool isInertial();
    bool isInertial(int nAgentId);
    void SetInertialSensor();
    void SetInertialSensor(int nAgentId);
    void SetImuInitialized();
    void SetImuInitialized(int nAgentId);
    bool isImuInitialized();
    bool isImuInitialized(int nAgentId);

    // Function for garantee the correction of serialization of this object
    void PreSave();
    void PostLoad();

    void SetKeyFrameDababase(KeyFrameDatabase* pKFDB);
    KeyFrameDatabase* GetKeyFrameDatabase();

    void SetORBVocabulary(ORBVocabulary* pORBVoc);
    ORBVocabulary* GetORBVocabulary();

    long unsigned int GetNumLivedKF();

    long unsigned int GetNumLivedMP();

    int GetNumAgentContext();




protected:
    //Agent related context members
    Agent* mpCurrentAgent;
    std::map<int, Map*> mMapAgentMap;
    std::map<int, std::vector<Map*>> mMapAgentMapVector;
    std::map<int, unsigned long int> mMapAgentLastInitFKidMap;
    std::map<int, int> mMapNumAgentMaps; //This member is created solely for aiding tracking initialization
    //Member variables that need to correspond to a unique agent
    Map* mpCurrentMap;
    unsigned long int mnLastInitKFidMap;

    //Member variables that can be shared between agents
    std::set<Map*> mspMaps;
    std::set<Map*> mspBadMaps;
    // Its necessary change the container from set to vector because libboost 1.58 and Ubuntu 16.04 have an error with this cointainer
    std::vector<Map*> mvpBackupMaps;

    std::vector<GeometricCamera*> mvpCameras;
    std::vector<KannalaBrandt8*> mvpBackupCamKan;
    std::vector<Pinhole*> mvpBackupCamPin;

    //Pinhole testCam;
    std::mutex mMutexAtlas;
    std::recursive_mutex mRMutexAtlas;
    Viewer* mpViewer;
    bool mHasViewer;

    // Class references for the map reconstruction from the save file
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

}; // class Atlas

} // namespace ORB_SLAM3

#endif // ATLAS_H
