# CoORBSLAM3

## About 
This repository contains a modified version of ORB SLAM 3 that enables multiple agents to feed data into a centralized server. 

## Development Status
Multi agent SLAM is currently not possible yet. The only changes is how ORBSLAM3 ingests new frames from a different ROSNODE 

## Dependencies 
The same dependencies are reflected from ORB SLAM 3. 
Although if you are not using OpenCV 4 you may run into build problems. The ares you will need to fix are:
1. Ensure `CMakeLists.txt` is finding the right version of OpenCV for you.
2. Ensure the thirdparty libraries DBoW and g2o are built using the correct OpenCV version. You can inspect their `CMakeLists.txt` file as well. 
3. Ensure your vision_opencv package is compatible with your version of OpenCV. 

## Installation 
1. Clone this repo into your catkin workspace src directory i.e. `~/catkin_ws/src`
2. Build it with `catkin_make` (or `catkin build`)
3. Copy the Vocabulary folder (from ORB SLAM 3) into the root of this repository i.e. ~/catkin_ws/src/CoORBSLAM3
4. Launch `roscore` 
5. Launch CoORBSLAM server using: Launch `rosrun CoORBSLAM3 CoORBSLAM3_mono <Vocabulary File> <Parameter File>`
   
   e.g. `rosrun CoORBSLAM3 CoORBSLAM3_mono ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml`
6. Launch the agent that feeds images 
   `rosrun CoORBSLAM3 agent_image <int::AgentId> <String::Path to Image Frames>`
   e.g:  `rosrun CoORBSLAM3 agent_image 2021 "~/data/imgs/"`

7. After launching your first Agent you can continue to launch additional agents with unique integer AgentIDs.

