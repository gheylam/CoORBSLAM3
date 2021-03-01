cmake_minimum_required(VERSION 3.0.2)
project(CoORBSLAM3)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(OpenCV 4 REQUIRED)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})
MESSAGE("PROJECT SOURCE DIR:")
MESSAGE(${PROJECT_SOURCE_DIR})

## Defining Dependent CMAKE Packages
find_package(Eigen3 REQUIRED 3.0)
find_package(Pangolin REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        image_transport
        cv_bridge
        sensor_msgs
        OpenCV
        )

MESSAGE("EIGEN INCLUDE PATH:")
MESSAGE(${EIGEN3_INCLUDE_DIRS})
MESSAGE("Pangolin INLUCDE PATH:")
MESSAGE(${Pangolin_INCLUDE_DIRS})

MESSAGE("CATKIN GLOBAL BIN DEST")
message(STATUS "OpenCV library status:")
message(STATUS "config: ${OpenCV_DIR}")
message(STATUS "version: ${OpenCV_VERSION}")
message(STATUS "libraries: ${OpenCV_LIBRARIES}")
message(STATUS "include path: ${OpenCV_INCLUDE_DIRS}")
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include/CameraModels
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}

)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
#add_message_files(
#    FILES
#   Message1.msg
#   Message2.msg
#)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   NewAgentFeed.srv
   NewAgentRequest.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
)

catkin_package(
        INCLUDE_DIRS include ${EIGEN_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS}
        LIBRARIES corbslam3
        CATKIN_DEPENDS roscpp std_msgs image_transport cv_bridge sensor_msgs
        #DEPENDS
)

add_library(
        ${PROJECT_NAME}
        ## source files
        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Atlas.cc
        src/Map.cc
        src/MapDrawer.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Initializer.cc
        src/Viewer.cc
        src/ImuTypes.cc
        src/G2oTypes.cc
        src/CameraModels/Pinhole.cpp
        src/CameraModels/KannalaBrandt8.cpp
        src/OptimizableTypes.cpp
        src/MLPnPsolver.cpp
        src/TwoViewReconstruction.cc

        ## Header files
        include/System.h
        include/Tracking.h
        include/LocalMapping.h
        include/LoopClosing.h
        include/ORBextractor.h
        include/ORBmatcher.h
        include/FrameDrawer.h
        include/Converter.h
        include/MapPoint.h
        include/KeyFrame.h
        include/Atlas.h
        include/Map.h
        include/MapDrawer.h
        include/Optimizer.h
        include/PnPsolver.h
        include/Frame.h
        include/KeyFrameDatabase.h
        include/Sim3Solver.h
        include/Initializer.h
        include/Viewer.h
        include/ImuTypes.h
        include/G2oTypes.h
        include/CameraModels/GeometricCamera.h
        include/CameraModels/Pinhole.h
        include/CameraModels/KannalaBrandt8.h
        include/OptimizableTypes.h
        include/MLPnPsolver.h
        include/TwoViewReconstruction.h
        src/ImgFrame.cpp include/ImgFrame.h src/Agent.cpp include/Agent.h src/LoopClosingManager.cpp include/LoopClosingManager.h)

set(THIRDPARTY_LIBS
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${OpenCV_LIBS}
        )

set(EXTERNAL_LIBRARIES
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        )




## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)




###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


###########
## Build ##
###########

## Setting build locations
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/debug)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/release)

## Specify additional locations of header files
## Your package locations should be listed before other locations

message(catkin include dir: ${catkin_INCLUDE_DIRS})
message(caktin123libraries: ${catkin_LIBRARIES})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/collaborative_orbslam3.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/collaborative_orbslam3_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        -lboost_serialization
        -lcrypto
        )

add_executable(CoORBSLAM3_mono src/corbslam3_mono.cpp)
target_link_libraries(CoORBSLAM3_mono ${PROJECT_NAME})
add_dependencies(CoORBSLAM3_mono CoORBSLAM3_generate_messages_cpp CoORBSLAM3_gencpp)

add_executable(agent_image src/agent_image.cpp)
target_link_libraries(agent_image ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(agent_image CoORBSLAM3_generate_messages_cpp CoORBSLAM3_gencpp)

add_executable(agent_node src/AgentNode.cpp)
target_link_libraries(agent_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(agent_node CoORBSLAM3_generate_messages_cpp CoORBSLAM3_gencpp)

