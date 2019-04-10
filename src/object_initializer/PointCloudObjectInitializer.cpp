//
// Created by kandithws on 17/3/2562.
//

#include <object_initializer/PointCloudObjectInitializer.h>
#include <random>
#include "utils/Config.h"
#include <pcl/common/transforms.h>


typedef pcl::PointXYZRGBL PointT;

namespace ORB_SLAM2 {
PointCloudObjectInitializer::PointCloudObjectInitializer() {
    // TODO : Config this
    mCloudSORFilter.setMeanK(Config::getInstance().ObjectInitializerParams().mean_k);
    mCloudSORFilter.setStddevMulThresh(Config::getInstance().ObjectInitializerParams().std_dev_mul_th);
    mbProject2d = Config::getInstance().ObjectInitializerParams().project_2d_outlier;
}

Cuboid PointCloudObjectInitializer::CuboidFromPointCloud(pcl::PointCloud<PointT>::Ptr cloud) {

    Cuboid cuboid;
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
    /// the signs are different and the box doesn't get correctly oriented in some cases.

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
    pcl::PointCloud<PointT>::Ptr cloudPointsProjected(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform, originates @ center of the cuboid

    // TODO: try to consistent z vector

    const Eigen::Quaternionf bboxQuaternion(
            eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    const Eigen::Vector3d scale = {(maxPoint.x - minPoint.x) / 2.0,
                                   (maxPoint.y - minPoint.y) / 2.0,
                                   (maxPoint.z - minPoint.z) / 2.0};

    cuboid.setRotation(bboxQuaternion.cast<double>());
    cuboid.setTranslation(bboxTransform.cast<double>());
    cuboid.setScale(scale);

    return cuboid;
}

void PointCloudObjectInitializer::InitializeObjects(KeyFrame *pKeyframe, Map *pMap) {

    auto vPredictedObjects = pKeyframe->GetObjectPredictions();
    // Retrieve neighbor keyframes in covisibility graph
    std::vector<int> vAssociatedCount(vPredictedObjects.size(), 0); // use int for debuging
    int nn = 10;
    //if(mbMonocular)
    //    nn=20;
    const vector<KeyFrame *> vpNeighKFs = pKeyframe->GetBestCovisibilityKeyFrames(nn);

    for (vector<KeyFrame *>::const_iterator vit_kf = vpNeighKFs.begin(), vend_kf = vpNeighKFs.end();
         vit_kf != vend_kf; vit_kf++) {

        if ((*vit_kf)->mvpMapObjects.empty()) {
            continue;
        }
        std::vector<bool> vCovisKFAssociatedFound(vPredictedObjects.size(), false);
        // TODO: Object Association
        SPDLOG_INFO("HELLO WORLD! KF:{}", pKeyframe->mnId);
        for (auto &pMO : (*vit_kf)->mvpMapObjects) {
            cv::Rect bb;
            // pMO->IsPositiveToKeyFrame(pKeyframe)
            if (pMO->IsPositiveToKeyFrame(pKeyframe)
            && pMO->GetProjectedBoundingBox(pKeyframe, bb)) { // Make sure
                // Find label match with nearest center
                double min_dist = std::numeric_limits<double>::max();
                long int min_dist_idx = -1;
                // TODO -- Q? should bb box center limited inside image or actual? (could be outside)
                cv::Point2f bb_center = (bb.tl() + bb.br()) * 0.5;
                for (size_t i = 0; i < vPredictedObjects.size(); i++) {
                    if (vCovisKFAssociatedFound[i]){
                        // Skip matched objects in the current co-visibility keyframe
                        continue;
                    }

                    // Find nearest bounding box center (we shouldn't use IOU because it may not intersec just yet)
                    if (vPredictedObjects[i]._label == pMO->mLabel) {
                        auto dist = Point2DDistance(bb_center, vPredictedObjects[i].GetCentroid2D());
                        // TODO -- add max image distance allow
                        SPDLOG_INFO("BB Center dist idx:{}, dist={}", i, dist);
                        if (dist < min_dist){
                            min_dist = dist;
                            min_dist_idx = i;
                        }
                    }
                }

                // Found match
                if (min_dist_idx > -1){
                    vAssociatedCount[min_dist_idx]++;
                    vCovisKFAssociatedFound[min_dist_idx] = true;
                    // TODO -- Add Observations info
                    SPDLOG_INFO("Associating Observation! idx={} of KFID={}, to ObjID={}",
                            min_dist_idx,
                            (*vit_kf)->mnId,
                            pMO->mnId);
                    pMO->AddObservation(pKeyframe, min_dist_idx);
                    pKeyframe->AddMapObject(pMO, min_dist_idx);
                    pMap->AddMapObject(pMO);
                }
            }
        }
    }


    SPDLOG_DEBUG("----Init Object for KF:  {} -----", pKeyframe->mnId);
    for (size_t i = 0; i < vPredictedObjects.size(); i++) {
        if (vAssociatedCount[i] > 0)
            continue;

        auto &pred = vPredictedObjects[i];
        // if the keyframe does not have object then continue
        auto vObjMapPoints = pKeyframe->GetMapPointsInBoundingBox(pred.box());
        SPDLOG_DEBUG("Obj {}: Total Point {}", i, vObjMapPoints.size());

        if (vObjMapPoints.size() < 10) {
            continue; // Too few points for calculation, TODO: Set as parameters
        }

        for (auto &pMP : vObjMapPoints) {
            pMP->SetPointColor(0x00FF0000); // For Debuging
        }

        auto cloudObject = PCLConverter::toPointCloud(vObjMapPoints);


        // -  PreProcessing, i.e. select only a maingroup of pointcloud (no need other processing)

        if (mbProject2d) {
            // TODO -- fix this
            SPDLOG_WARN("[mbProject2d] This feature is still on debugging");
            // Transform all points to local frame
            auto kf_tf_mat = pKeyframe->GetPose();
            Eigen::Affine3f kf_tf;
            PCLConverter::makeAffineTf(kf_tf_mat, kf_tf);

            Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
            projectionTransform.block<3, 3>(0, 0) = kf_tf.rotation().matrix().transpose();
            projectionTransform.block<3, 1>(0, 3) = -1.f * kf_tf.translation();
            pcl::PointCloud<PointT>::Ptr cloudPointsProjected(new pcl::PointCloud<PointT>);
            pcl::transformPointCloud(*cloudObject, *cloudObject, projectionTransform);

            // pcl::transformPointCloud(*cloudObject, *cloudObject, kf_tf);

            // projection
            auto end = cloudObject->end();
            for (auto it = cloudObject->begin(); it != end; it++) {
                // For each point compute relative tf
                // pcl::transformPoint(*it, *it, );
                it->y = 0; // Onto ZX plane
            }

        }

        pcl::PointCloud<PointT>::Ptr cloudSegmented(new pcl::PointCloud<PointT>);
        std::vector<MapPoint *> vFilteredMapPoints;
        std::vector<int> vFilteredMapPointsIndices;
        mCloudSORFilter.setInputCloud(cloudObject);
        mCloudSORFilter.setNegative(false); // ensure
        auto start_time = utils::time::time_now();
        //mCloudSORFilter.filter(*cloudSegmented);
        mCloudSORFilter.filter(vFilteredMapPointsIndices);
        SPDLOG_DEBUG("Filtering time {}s", utils::time::time_diff_from_now_second(start_time));
        // try to filter, if it is fails on positive, try to get negative
        if (vFilteredMapPointsIndices.empty()) {
            SPDLOG_WARN("Set Negative Filter");
            mCloudSORFilter.setNegative(true); // ensure
            vFilteredMapPoints.clear();
            mCloudSORFilter.filter(vFilteredMapPointsIndices);
        }


        PCLConverter::filterVector<MapPoint *>(vFilteredMapPointsIndices, vObjMapPoints, vFilteredMapPoints);
        // - Calculate Cuboid parameters
        auto start_time2 = utils::time::time_now();

        auto inliers = PCLConverter::toPointCloud(vFilteredMapPoints);

        auto cuboid = CuboidFromPointCloud(inliers);
        SPDLOG_DEBUG("Cuboid calculation time {}s", utils::time::time_diff_from_now_second(start_time2));
        auto scale = cuboid.mScale;
        SPDLOG_DEBUG("Cuboid scale x:{}, y:{}, z:{}", scale[0] * 2.0, scale[1] * 2.0, scale[2] * 2.0);

        uint32_t color = (static_cast<uint32_t>(std::rand() % 255) << 16) |
                         (static_cast<uint32_t>(std::rand() % 255) << 8) |
                         (static_cast<uint32_t>(std::rand() % 255));

        for (auto &pMP : vFilteredMapPoints) {
            pMP->SetPointColor(color); // For Debuging
        }

        // Create new MapObject -- pattern from LocalMapping.cc MapPoint!
        MapObject *pMO = new MapObject(cuboid, pred._label, pKeyframe, pMap);
        pMO->AddObservation(pKeyframe, i);
        pKeyframe->AddMapObject(pMO, i);
        pMap->AddMapObject(pMO);

    }
}

}