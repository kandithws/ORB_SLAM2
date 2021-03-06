cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE RelWithDebInfo)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

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

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(Boost REQUIRED)
find_package(OpenCV 3.4.4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(VTK 6 REQUIRED)
find_package(PCL 1.9 REQUIRED)

find_package(Protobuf REQUIRED)
find_package(GRPC REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# Temporary fix for CUDA dep, need to remove CUDA from OpenCV + PANGOLIN + PCL later
find_package(CUDA)
if (CUDA_FOUND)
   include_directories(${CUDA_INCLUDE_DIRS})
endif ()

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../src
${PROJECT_SOURCE_DIR}/../../../include
${Pangolin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# somebug
list(REMOVE_ITEM PCL_LIBRARIES
        "vtkGUISupportQt"
        "vtkGUISupportQtOpenGL"
        "vtkGUISupportQtSQL"
        "vtkGUISupportQtWebkit"
        "vtkViewsQt"
        "vtkRenderingQt"
        "vtkRenderingMatplotlib")

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
        ${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
)

# Node for monocular camera
rosbuild_add_executable(VIORB
src/ros_vio.cc
src/MsgSync/MsgSynchronizer.cpp
src/MsgSync/MsgSynchronizer.h
)

target_link_libraries(VIORB
${LIBS}
)

rosbuild_add_executable(stereo
        src/ros_vio_stereo.cc
        src/MsgSync/StereoMsgSynchronizer.cpp
        src/MsgSync/StereoMsgSynchronizer.h
        )

target_link_libraries(stereo
        ${LIBS}
        )

rosbuild_add_executable(bag_view_test
        src/bag_view_test.cpp
        )

target_link_libraries(bag_view_test
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        )


