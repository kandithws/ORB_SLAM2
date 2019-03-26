//
// Created by kandithws on 17/3/2562.
//

#include <object_initializer/PointCloudObjectInitializer.h>
#include <random>


typedef pcl::PointXYZRGBL PointT;

namespace ORB_SLAM2 {
PointCloudObjectInitializer::PointCloudObjectInitializer() {
    // TODO : Config this
    mCloudSORFilter.setMeanK(8);
    mCloudSORFilter.setStddevMulThresh(0.8);
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
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform, originates @ center of the cuboid

    // TODO: try to consistent z vector

    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    const Eigen::Vector3d scale = { (maxPoint.x - minPoint.x) / 2.0,
                                    (maxPoint.y - minPoint.y) / 2.0,
                                    (maxPoint.z - minPoint.z) / 2.0 };

    cuboid.setRotation(bboxQuaternion.cast<double>());
    cuboid.setTranslation(bboxTransform.cast<double>());
    cuboid.setScale(scale);

    return cuboid;
}

void PointCloudObjectInitializer::InitializeObjects(KeyFrame *pKeyframe, Map *pMap) {

    auto vPredictedObjects = pKeyframe->GetObjectPredictions();
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 10;
    //if(mbMonocular)
    //    nn=20;
    const vector<KeyFrame*> vpNeighKFs = pKeyframe->GetBestCovisibilityKeyFrames(nn);


    for(vector<KeyFrame*>::const_iterator vit_kf=vpNeighKFs.begin(), vend_kf=vpNeighKFs.end(); vit_kf!=vend_kf; vit_kf++){
        if ( (*vit_kf)->mvObject.size() == 0)
                continue;

            // TODO: Object Association

    }

    int i = 0;
    SPDLOG_DEBUG("----Init Object for KF:  {} -----", pKeyframe->mnId);
    for (auto &pred : vPredictedObjects){
        // if the keyframe does not have object then continue
        auto vObjMapPoints = pKeyframe->GetMapPointsInBoundingBox(pred.box());
        SPDLOG_DEBUG("Obj {}: Total Point {}", i, vObjMapPoints.size());

        if (vObjMapPoints.size() < 8){
            continue; // Too few points for calculation, TODO: Set as parameters
        }

        for (auto &pMP : vObjMapPoints){
            pMP->SetPointColor(0x00FF0000); // For Debuging
        }

        auto cloudObject = PCLConverter::toPointCloud(vObjMapPoints);

#ifdef ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_DEBUG
        // -  Get pcl::PointCloud
        std::string outname = "/home/kandithws/debug_clouds/kf_"
                + std::to_string(pKeyframe->mnId)
                + "_obj_" + std::to_string(i)
                + "_l"+ std::to_string(pred._label)
                + ".pcd";
        mCloudDebugWriter.write(outname, *cloudObject);
#endif

        // -  PreProcessing, i.e. select only a maingroup of pointcloud (no need other processing)
        // use pcl::filter.getRemovedIndices()

        pcl::PointCloud<PointT>::Ptr cloudSegmented (new pcl::PointCloud<PointT>);
        std::vector<MapPoint*> vFilteredMapPoints;
        mCloudSORFilter.setInputCloud(cloudObject);
        mCloudSORFilter.setNegative(false); // ensure
        auto start_time = utils::time::time_now();
        mCloudSORFilter.filter(*cloudSegmented);
        SPDLOG_DEBUG("Filtering time {}s", utils::time::time_diff_from_now_second(start_time));
        // try to filter, if it is fails on positive, try to get negative
        if (cloudSegmented->empty()){
            SPDLOG_WARN("Set Negative Filter");
            mCloudSORFilter.setNegative(true); // ensure
            mCloudSORFilter.filter(*cloudSegmented);
        }

        PCLConverter::filterVector<MapPoint*>(*mCloudSORFilter.getIndices(), vObjMapPoints, vFilteredMapPoints);
        // - Calculate Cuboid parameters
        auto start_time2 = utils::time::time_now();
        auto cuboid = CuboidFromPointCloud(cloudSegmented);
        SPDLOG_DEBUG("Cuboid calculation time {}s", utils::time::time_diff_from_now_second(start_time2));
        auto scale = cuboid.mScale;
        SPDLOG_DEBUG("Cuboid scale x:{}, y:{}, z:{}", scale[0] * 2.0, scale[1]  * 2.0, scale[2]  * 2.0);

        for (auto &pMP : vFilteredMapPoints){
//            uint32_t color = ( static_cast<uint32_t>(std::rand() % 255) << 16) |
//                    ( static_cast<uint32_t>(std::rand() % 255) << 8) |
//                    ( static_cast<uint32_t>(std::rand() % 255));

            pMP->SetPointColor(0x0000FF00); // For Debuging
        }
        // Create new MapObject

        // Insert to Map & addObservations (KF)

        i++;
    }
}

}