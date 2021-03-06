cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O2 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O2 -march=native")

add_compile_options(-std=c++14)
add_definitions(-DCOMPILEDWITHC11) # for in example programs
# Check C++11 or C++0x support
# include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if (COMPILER_SUPPORTS_CXX11)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#    add_definitions(-DCOMPILEDWITHC11)
#    message(STATUS "Using flag -std=c++11.")
#elseif (COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#    add_definitions(-DCOMPILEDWITHC0X)
#    message(STATUS "Using flag -std=c++0x.")
#else ()
#    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
#endif ()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


find_package(OpenCV 3.4.4 REQUIRED)
find_package(Eigen3 3.2.10 REQUIRED)
find_package(Pangolin REQUIRED)
# find_package(VTK 8 REQUIRED)
find_package(PCL 1.7 REQUIRED)

# In cmake modules
find_package(Protobuf REQUIRED)
find_package(GRPC REQUIRED)
find_package(Cholmod REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# Temporary fix for CUDA dep, need to remove CUDA from OpenCV + PANGOLIN + PCL later
find_package(CUDA)
if (CUDA_FOUND)
    include_directories(${CUDA_INCLUDE_DIRS})
endif ()

# BUILDING PROTOBUF Messages
set(PROTOS ${CMAKE_CURRENT_SOURCE_DIR}/protos/detection.proto ${CMAKE_CURRENT_SOURCE_DIR}/protos/detection_v2.proto)
set(PROTO_SRC_DIR ${PROJECT_SOURCE_DIR}/include/grpc)
include_directories(${PROTO_SRC_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_SRC_DIR} ${PROTOS})
grpc_generate_cpp(GRPC_SRCS GRPC_HDRS ${PROTO_SRC_DIR} ${PROTOS})

# ${Pangolin_INCLUDE_DIRS}
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${CHOLMOD_INCLUDE_DIR}
)


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(UTILS_SRC_FILES src/utils/Config.cpp
        include/utils/Config.h
        include/utils/time.h include/utils/PCLConverter.h
        src/utils/PCLConverter.cpp
        include/utils/matrix_utils.h
        src/utils/matrix_utils.cpp)


set(DNN_SRC_FILES
        include/dnn/BaseObjectDetector.h
        include/dnn/CVObjectDetector.h
        src/dnn/BaseObjectDetector.cpp
        src/dnn/CVObjectDetector.cpp include/dnn/GrpcObjectDetector.h
        ${PROTO_SRCS} ${GRPC_SRCS} src/dnn/GrpcObjectDetector.cpp
        include/dnn/GrpcObjectDetectorV2.h src/dnn/GrpcObjectDetectorV2.cpp)


set(OBJECT_INIT_SRC_FILES
        include/object_initializer/IObjectInitializer.h
        include/object_initializer/PointCloudObjectInitializer.h
        src/object_initializer/PointCloudObjectInitializer.cpp)

set(VIORB_SRC_FILES
        src/imu/complementary_filter.cpp
        src/imu/g2o_types.cpp
        src/imu/IMUData.cpp
        src/imu/IMUPreintegrator.cpp
        src/imu/NavState.cpp
        src/imu/so3.cpp
        src/IMUOptimizer.cpp)

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
        src/Viewer.cc
        src/g2o_Object.cpp
        include/MapObject.h src/MapObject.cpp include/Cuboid.h src/Cuboid.cpp src/ObjectOptimizer.cpp)

add_library(${PROJECT_NAME} SHARED
        ${ORBSLAM2_SRC_FILES}
        ${UTILS_SRC_FILES}
        ${DNN_SRC_FILES}
        ${OBJECT_INIT_SRC_FILES}
        ${VIORB_SRC_FILES}
        )

#  ${Pangolin_LIBRARIES}
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        ${Pangolin_LIBRARIES}
        ${PCL_LIBRARIES}
        gRPC::grpc++_reflection
        protobuf::libprotobuf
        ${CHOLMOD_LIBRARIES}
        )

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
        Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME} ${PCL_LIBRARIES})
#
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)
#
add_executable(stereo_kitti
  Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME} ${PCL_LIBRARIES})

add_executable(stereo_tum
        Examples/Stereo/stereo_tum.cc)
target_link_libraries(stereo_tum ${PROJECT_NAME} ${PCL_LIBRARIES})

#
#add_executable(stereo_euroc
#  Examples/Stereo/stereo_euroc.cc)
#target_link_libraries(stereo_euroc ${PROJECT_NAME} ${PCL_LIBRARIES})
#
#
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
  Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME} ${PCL_LIBRARIES})

add_executable(mono_kitti
  Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME} ${PCL_LIBRARIES})
#
#add_executable(mono_euroc
#  Examples/Monocular/mono_euroc.cc)
#target_link_libraries(mono_euroc ${PROJECT_NAME} ${PCL_LIBRARIES})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/VIORB)

add_executable(viorb_euroc
        Examples/VIORB/viorb_euroc.cpp)
target_link_libraries(viorb_euroc ${PROJECT_NAME} ${PCL_LIBRARIES})

