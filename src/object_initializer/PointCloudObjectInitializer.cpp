//
// Created by kandithws on 17/3/2562.
//

#include <object_initializer/PointCloudObjectInitializer.h>


namespace ORB_SLAM2 {
PointCloudObjectInitializer::PointCloudObjectInitializer() {

}

void PointCloudObjectInitializer::InitializeObjects(ORB_SLAM2::KeyFrame *pKeyframe, ORB_SLAM2::Map *pMap) {
    auto vPredictedObjects = pKeyframe->GetObjectPredictions();
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 10;
    //if(mbMonocular)
    //    nn=20;
    const vector<KeyFrame*> vpNeighKFs = pKeyframe->GetBestCovisibilityKeyFrames(nn);

    // We have 2 solutions to this
    // 1. Initialize objects first
    //      1.1 Get all mappoints in BB, create temporary Object
    //      1.2 Perform Object association and new object decision
    //
    // 2. Get Approximate Object center only
    //      2.1 Triangulate point from BB center, raytracing and get pointcloud of something ( pcl::octree::OctreePointCloudSearch)
    //      2.2 find nearest object and new object decision

    int i = 0;
    SPDLOG_DEBUG("----Init Object for KF:  {} -----", pKeyframe->mnId);
    for (auto &pred : vPredictedObjects){
        // if the keyframe does not have object then continue
        auto vObjMapPoints = pKeyframe->GetMapPointsInBoundingBox(pred.box());
        SPDLOG_DEBUG("Obj {}: Total Point {}", i++, vObjMapPoints.size());

        // -  Get pcl::PointCloud

        // -  PreProcessing, i.e. select only a maingroup of pointcloud (no need other processing)
        //

        // - Calculate Cuboid parameters (cv::Mat)

        /*
        for(vector<KeyFrame*>::const_iterator vit_kf=vpNeighKFs.begin(), vend_kf=vpNeighKFs.end(); vit_kf!=vend_kf; vit_kf++){
            if ( (*vit_kf)->mvObject.size() == 0)
                continue;

            // TODO: Object Association

            // If Object Poses are fused, Culling map points
        }
        */

    }
}

}