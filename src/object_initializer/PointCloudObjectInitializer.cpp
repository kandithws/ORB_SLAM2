//
// Created by kandithws on 17/3/2562.
//

#include <object_initializer/PointCloudObjectInitializer.h>
#include <random>
#include "utils/Config.h"
#include <pcl/common/transforms.h>
#include <utils/vector_utils.h>

typedef pcl::PointXYZRGBL PointT;

namespace ORB_SLAM2 {
PointCloudObjectInitializer::PointCloudObjectInitializer() {
    // TODO : Config this
    mCloudSORFilter.setMeanK(Config::getInstance().ObjectInitializerParams().mean_k);
    mCloudSORFilter.setStddevMulThresh(Config::getInstance().ObjectInitializerParams().std_dev_mul_th);
    mbProject2d = Config::getInstance().ObjectInitializerParams().project_2d_outlier;
    mbUseMask = Config::getInstance().ObjectInitializerParams().use_mask;
    mbUseStatRemoveOutlier = Config::getInstance().ObjectInitializerParams().use_stat_rm_outlier;
    //mbUseMask = true;
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

    // ------------- Object Association -------------------
    const vector<KeyFrame *> vpNeighKFs = pKeyframe->GetBestCovisibilityKeyFrames(nn);

    for (vector<KeyFrame *>::const_iterator vit_kf = vpNeighKFs.begin(), vend_kf = vpNeighKFs.end();
         vit_kf != vend_kf; vit_kf++) {

        if ((*vit_kf)->mvpMapObjects.empty()) {
            continue;
        }

        std::vector<bool> vCovisKFAssociatedFound(vPredictedObjects.size(), false);
        std::vector<MapObject*> vpMapObjects = (*vit_kf)->mvpMapObjects; // TODO -- MUTEX

        for(size_t obj_idx=0; obj_idx < vpMapObjects.size(); obj_idx++) {
            MapObject* pMO = vpMapObjects[obj_idx];
            cv::Rect bb;

            if(!pMO){
                //TODO -- fix this, this mean all predictions doesn't either create a new object or match others
                continue;
            }

            if (pMO->IsPositiveToKeyFrame(pKeyframe)
            && pMO->GetProjectedBoundingBox(pKeyframe, bb)) {
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
                    if (vPredictedObjects[i]->_label == pMO->mLabel) {
                        auto dist = Point2DDistance(bb_center, vPredictedObjects[i]->GetCentroid2D());
                        // TODO -- add max image distance allow
                        //SPDLOG_INFO("BB Center dist idx:{}, dist={}", i, dist);
                        if (dist < min_dist){
                            min_dist = dist;
                            min_dist_idx = i;
                        }
                    }
                }

                // Found match
                if (min_dist_idx > -1){
                    vAssociatedCount[min_dist_idx]++;
                    assert(!vCovisKFAssociatedFound[min_dist_idx]);
                    vCovisKFAssociatedFound[min_dist_idx] = true;
                    //SPDLOG_INFO("Associating Observation! idx={} of KFID={}, to ObjID={}",
                    //        min_dist_idx,
                    //        (*vit_kf)->mnId,
                    //        pMO->mnId);
                    pMO->AddObservation(pKeyframe, min_dist_idx);
                    pKeyframe->AddMapObject(pMO, min_dist_idx);
                    pMap->AddMapObject(pMO);
                }
            }
        }
    }

    // ------------- Init objects where there is no association-------------------

    SPDLOG_DEBUG("----Init Object for KF:  {} -----", pKeyframe->mnId);
    for (size_t i = 0; i < vPredictedObjects.size(); i++) {
        if (vAssociatedCount[i] > 0)
            continue;

        auto &pred = *vPredictedObjects[i];
        const auto box = pred.box();
        int bboxSizeThresh = min(pKeyframe->mnMaxX - pKeyframe->mnMinX, pKeyframe->mnMaxY - pKeyframe->mnMinY) / 10;

        if (box.width < bboxSizeThresh || box.height < bboxSizeThresh){
            continue;
        }

        // if the keyframe does not have object then continue
        if(mbUseMask && pred._mask_type == PredictedObject::MASK_TYPE::NO_MASK)
            SPDLOG_WARN("Configure to use mask, but no mask provided");

        auto vObjMapPoints = mbUseMask && (pred._mask_type != PredictedObject::MASK_TYPE::NO_MASK) ?
                pKeyframe->GetMapPointsInMask(box, pred._mask, pred._mask_type) : pKeyframe->GetMapPointsInBoundingBox(box);

//        if (pred._mask_type != PredictedObject::MASK_TYPE::NO_MASK){
//            std::string outdebug = "debug/mask_"+ Config::getInstance().getLabelName(pred._label)
//                    + "kf_" + std::to_string(pKeyframe->mnId) + "obj_" + std::to_string(i) + ".png";
//
//            cv::Mat drawn_mask;
//            SPDLOG_INFO("I AM HERE!!!!!!");
//            pKeyframe->DrawDebugPointsInMask(drawn_mask, box, pred._mask, pred._mask_type);
//            cv::imwrite(outdebug, drawn_mask);
//        }
        SPDLOG_DEBUG("Obj {}: Total Point {}", i, vObjMapPoints.size());

        if (vObjMapPoints.size() < 8) {
            continue; // Too few points for calculation, TODO: Set as parameters
        }


        for (auto &pMP : vObjMapPoints) {
            pMP->SetPointColor(0x00FF0000); // For Debuging
        }

        std::vector<MapPoint *> vFilteredMapPoints;
        if (mbUseStatRemoveOutlier) {
            auto cloudObject = PCLConverter::toPointCloud(vObjMapPoints);
            // -  PreProcessing, i.e. select only a maingroup of pointcloud (no need other processing)
            if (mbProject2d) {
                // Instead of performming SOR on global frame
                // we present points onto ZX plane so that SOR would find outliers
                // In 2D perspective along the camera observation ray (not 3D)
                // This would help to keep inliers for large tall that may have sparse
                // point cloud along its height direction + outliers group vs object pointgroup would be more clutered
                // TODO -- Optimize this!
                // Transform all points to local frame
                auto kf_tf_mat = pKeyframe->GetPose();
                Eigen::Affine3f kf_tf;
                PCLConverter::makeAffineTf(kf_tf_mat, kf_tf);
                // projection
                auto end = cloudObject->end();
                for (auto it = cloudObject->begin(); it != end; it++) {
                    // For each point compute relative tf
                    Eigen::Vector3f tmp = { it->x, it->y, it->z};
                    Eigen::Vector3f out;
                    pcl::transformPoint(tmp, out, kf_tf);
                    it->x = out[0];
                    it->y = 0; // Onto ZX plane
                    it->z = out[2];
                }
            }
            pcl::PointCloud<PointT>::Ptr cloudSegmented(new pcl::PointCloud<PointT>);
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
        }
        else{
            vFilteredMapPoints = vObjMapPoints;
        }



        // - Calculate Cuboid parameters
        auto inliers = PCLConverter::toPointCloud(vFilteredMapPoints);
        auto cuboid = CuboidFromPointCloud(inliers);

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

    // TODO -- release current KF debugging image
    std::lock_guard<std::mutex> imglock(pKeyframe->mMutexImages);
    pKeyframe->mImGray.release();
    pKeyframe->mImColor.release();
}

}