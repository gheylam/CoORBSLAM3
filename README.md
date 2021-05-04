# CoORBSLAM3

## About 
This repository contains a modified version of ORB SLAM 3 that enables multiple agents to feed data into a centralized server. 

## Development Status
Multi agent SLAM is currently not possible yet. The only changes is how ORBSLAM3 ingests new frames from a different ROSNODE 

## Dependencies 
- ROS Melodic
- OpenCV4 
- Eigen3 
- Pangolin

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
   `rosrun CoORBSLAM3 agent_image <int::AgentId> <String::Path to Image Frames> <String::Path to Parameter File>`
   e.g:  `rosrun CoORBSLAM3 agent_image 2021 "./data/imgs/" "./EuRoC.yaml`
   An example parameter file is provided called  ROOM.YAML, both the server and the agent currently use the same
   parameter file. 
7. After launching your first Agent you can continue to launch additional agents with unique integer AgentIDs.

## Example Datasets 
	- You can use the EuRoC data which can be obtained [here](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
	- You can also use my random dataset of a room with ROOM.YAML parameter file [here](https://drive.google.com/file/d/1SLa1uzJEkkItUc2Yn_aN5i1Ulo1uD-6i/view?usp=sharing)
	
	- If you create you own dataset, you need to calibrate your camera and input the intrinsics and distortion parameters into your parameter file. (fx, fy, cx, cy, k1, k2, p1 and p2). You will also need to convert your images to greyscale first. 

## Fixing OpenCV3 conflicts with OpenCV4 

If you are planning to transition to OpenCV4 you will find yourself needing to build cv_bridge from source. 
Not doing so will cause conflicts with ROS melodic's preshipped OpenCV3 builds. To help you get starting I 
you build this forked version of the `vision_opencv` package by [BrutusTT](https://github.com/BrutusTT/vision_opencv). 

### Steps for building vision_opencv from source 
1. Clone the fork by BrutusTT into your catkin workspace source (i.e. `~/catkin_ws/src`)
2. Edit the following `CMakeLists.txt` in: 
   - `vision_opencv/cv_bridge/`
   - `vision_opencv/image_geometry/`
3. Build with `catkin_make` or `catkin build` (I found that `catkin_make` did not build correctly for me 
   but `catkin build` was able to.) You can find out more about catkin build [here](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html). 

### Changes to the vision_opencv CMakeLists.txt 
I have made the following changes to my CMakeLists.txt files referenced in the steps above:
- `set (CMAKE_CXX_STANDARD 11)`
- `find_package(OpenCV 4 REQUIRED...`

### Extra reading on support for OpenCV4 compatibility for vision_opencv

You can read up more on these issues to provide OpenCV4 compatibility [#272](https://github.com/ros-perception/vision_opencv/issues/272)
and pull requests [#259](https://github.com/ros-perception/vision_opencv/pull/259) and [#288](https://github.com/ros-perception/vision_opencv/pull/288).

For myself, I found that [fizyr-forks](https://github.com/fizyr-forks/vision_opencv/tree/opencv4) did not build for me. The errors I found from this fork 
are outlined in issue [#291](https://github.com/ros-perception/vision_opencv/issues/291).

### Problems with conflicting OpenCV libraries 

Some of the errors I came across include: 

`OpenCV Error: Assertion failed (tlsSlots.size() > slotIdx) in
releaseSlot, file
/build/opencv-L2vuMj/opencv-3.2.0+dfsg/modules/core/src/system.cpp,
line 1092 terminate called after throwing an instance of
'cv::Exception' what():
/build/opencv-L2vuMj/opencv-3.2.0+dfsg/modules/core/src/system.cpp:1092:
error: (-215) tlsSlots.size() > slotIdx in function releaseSlot`

`/usr/bin/ld: warning: libopencv_core.so.4.2, needed by /usr/local/lib/libopencv_highgui.so.4.5.0, may conflict with libopencv_core.so.3.2.`
