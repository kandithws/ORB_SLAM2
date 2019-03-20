cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

IF (NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF ()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  add_definitions(-DCOMPILEDWITHC11)
  message(STATUS "Using flag -std=c++11.")
elseif (COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCOMPILEDWITHC0X)
  message(STATUS "Using flag -std=c++0x.")
else ()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


find_package(OpenCV 3.4.4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
# find_package(VTK 8 REQUIRED)
find_package(PCL 1.9 REQUIRED)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# Temporary fix for CUDA dep, need to remove CUDA from OpenCV + PANGOLIN + PCL later
find_package(CUDA)
if (CUDA_FOUND)
 include_directories(${CUDA_INCLUDE_DIRS})
endif()

# ${Pangolin_INCLUDE_DIRS}
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
#   src/pcl_slam_visualizer/customized_pcl_visualizer/customized_interactor_style.cpp
#  src/pcl_slam_visualizer/customized_pcl_visualizer/customized_pcl_visualizer.cpp
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(UTILS_SRC_FILES src/utils/Config.cpp
  include/utils/Config.h
  include/utils/time.h include/utils/PCLConverter.h src/utils/PCLConverter.cpp)
set(PCL_EXTENDED_MODULE_SRC_FILES
  #  src/pcl_slam_visualizer/customized_pcl_visualizer/customized_interactor_style.cpp
  #  src/pcl_slam_visualizer/customized_pcl_visualizer/customized_pcl_visualizer.cpp
  src/pcl_slam_visualizer/pcl_slam_visualizer.cpp
  src/PCLViewer.cpp)

set(DNN_SRC_FILES
  include/dnn/BaseObjectDetector.h
  include/dnn/CVObjectDetector.h
  src/dnn/BaseObjectDetector.cpp
  src/dnn/CVObjectDetector.cpp)


set(OBJECT_INIT_SRC_FILES
  include/object_initializer/BaseObjectInitializer.h
  include/object_initializer/PointCloudObjectInitializer.h
        src/object_initializer/PointCloudObjectInitializer.cpp)

set(ORBSLAM2_SRC_FILES
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
  src/Map.cc
  src/MapDrawer.cc
  src/Optimizer.cc
  src/PnPsolver.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/Initializer.cc
  src/Viewer.cc include/MapObject.h src/MapObject.cpp)

add_library(${PROJECT_NAME} SHARED
  ${ORBSLAM2_SRC_FILES}
  ${UTILS_SRC_FILES}
  ${PCL_EXTENDED_MODULE_SRC_FILES}
  ${DNN_SRC_FILES}
  ${OBJECT_INIT_SRC_FILES}
  )

#  ${Pangolin_LIBRARIES}
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${Pangolin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

# Build examples

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)
#
add_executable(rgbd_tum
  Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME} ${PCL_LIBRARIES})
#
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)
#
#add_executable(stereo_kitti
#  Examples/Stereo/stereo_kitti.cc)
#target_link_libraries(stereo_kitti ${PROJECT_NAME} ${PCL_LIBRARIES})
#
#add_executable(stereo_euroc
#  Examples/Stereo/stereo_euroc.cc)
#target_link_libraries(stereo_euroc ${PROJECT_NAME} ${PCL_LIBRARIES})
#
#
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)
#
#add_executable(mono_tum
#  Examples/Monocular/mono_tum.cc)
#target_link_libraries(mono_tum ${PROJECT_NAME} ${PCL_LIBRARIES})
#
#add_executable(mono_kitti
#  Examples/Monocular/mono_kitti.cc)
#target_link_libraries(mono_kitti ${PROJECT_NAME} ${PCL_LIBRARIES})
#
#add_executable(mono_euroc
#  Examples/Monocular/mono_euroc.cc)
#target_link_libraries(mono_euroc ${PROJECT_NAME} ${PCL_LIBRARIES})

