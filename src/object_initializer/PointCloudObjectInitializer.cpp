//
// Created by kandithws on 17/3/2562.
//

#include <object_initializer/PointCloudObjectInitializer.h>
#include <random>
#include "utils/Config.h"
#include <pcl/common/transforms.h>
#include <utils/vector_utils.h>


//typedef pcl::PointXYZRGBL PointT;
typedef pcl::PointXYZRGB PointT;

namespace ORB_SLAM2 {
PointCloudObjectInitializer::PointCloudObjectInitializer(Map* pMap) : mpMap(pMap)
{
    // TODO : Config this
    mCloudSORFilter.setMeanK(Config::getInstance().ObjectInitializerParams().mean_k);
    mCloudSORFilter.setStddevMulThresh(Config::getInstance().ObjectInitializerParams().std_dev_mul_th);
    mbProject2d = Config::getInstance().ObjectInitializerParams().project_2d_outlier;
    mbUseMask = Config::getInstance().ObjectInitializerParams().use_mask;
    mbUseStatRemoveOutlier = Config::getInstance().ObjectInitializerParams().use_stat_rm_outlier;
    mOutlierFilterType = Config::getInstance().ObjectInitializerParams().outlier_filter_type;
    mOutlierFilterThreshold = Config::getInstance().ObjectInitializerParams().outlier_threshold;
    mProj.setModelType(pcl::SACMODEL_PLANE);
    mMatrixRotatePitch90 = Eigen::AngleAxisf(M_PI/2.0f, Eigen::Vector3f::UnitY()).toRotationMatrix();
    mbAccociateCentroid = Config::getInstance().ObjectInitializerParams().associate_centroid_only;
    auto associate = Config::getInstance().ObjectInitializerParams().associate_constraint;
    if (associate >=0)
        mAssociateConstraint = (uint8_t)associate;

    mAssociateTimeDiff = Config::getInstance().ObjectInitializerParams().associate_time_diff;
    mAssociateAngleDiff = Config::getInstance().ObjectInitializerParams().associate_angle_diff;
    mAssociateMax2DDist = Config::getInstance().ObjectInitializerParams().associate_max_2d_dist;
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


Cuboid* PointCloudObjectInitializer::CuboidFromPointCloudWithGravity(pcl::PointCloud<PointT>::Ptr cloud, const Eigen::Vector3f& gNormalized)  {

    Cuboid* cuboid = new Cuboid();
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud, pcaCentroid);

    // Create a plane from the gravity vector
    pcl::PointCloud<PointT>::Ptr cloud_projected (new pcl::PointCloud<PointT>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize(4);
    coefficients->values[0] = gNormalized[0];
    coefficients->values[1] = gNormalized[1];
    coefficients->values[2] = gNormalized[2];
    coefficients->values[3] = -( gNormalized.dot(pcaCentroid.head(3)));

    // Project to a plane
    // pcl::ProjectInliers<PointT> proj;
    // proj.setModelType (pcl::SACMODEL_PLANE);
    mProj.setInputCloud (cloud);
    mProj.setModelCoefficients (coefficients);
    mProj.filter (*cloud_projected);


    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud_projected, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
    /// the signs are different and the box doesn't get correctly oriented in some cases.

    // Make z-axis opposite with gravity vector
    if (eigenVectorsPCA.col(0).dot(gNormalized) <= 0)
        eigenVectorsPCA = eigenVectorsPCA * mMatrixRotatePitch90;
    else
        eigenVectorsPCA = eigenVectorsPCA * mMatrixRotatePitch90.transpose();

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

    const Eigen::Quaternionf bboxQuaternion(
            eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    const Eigen::Vector3d scale = {(maxPoint.x - minPoint.x) / 2.0,
                                   (maxPoint.y - minPoint.y) / 2.0,
                                   (maxPoint.z - minPoint.z) / 2.0};

    cuboid->setRotation(bboxQuaternion.cast<double>());
    cuboid->setTranslation(bboxTransform.cast<double>());
    cuboid->setScale(scale);
    return cuboid;
}

void PointCloudObjectInitializer::FilterMapPointsSOR(const vector<ORB_SLAM2::MapPoint *> &vObjMapPoints,
                                                     vector<ORB_SLAM2::MapPoint *> &vFilteredMapPoints,
                                                     bool bProject2D,
                                                     KeyFrame* pKeyFrame) {
    auto cloudObject = PCLConverter::toPointCloud(vObjMapPoints);
    // -  PreProcessing, i.e. select only a maingroup of pointcloud (no need other processing)
    if (bProject2D) {
        assert(pKeyFrame);
        // Instead of performming SOR on global frame
        // we present points onto ZX plane so that SOR would find outliers
        // In 2D perspective along the camera observation ray (not 3D)
        // This would help to keep inliers for large tall that may have sparse
        // point cloud along its height direction + outliers group vs object pointgroup would be more clutered
        // TODO -- Optimize this!
        // Transform all points to local frame
        auto kf_tf_mat = pKeyFrame->GetPose();
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

void PointCloudObjectInitializer::FilterMapPointsDistFromCentroid(const vector<ORB_SLAM2::MapPoint *> &vObjMapPoints,
                                                                  vector<ORB_SLAM2::MapPoint *> &vFilteredMapPoints,
                                                                  ORB_SLAM2::KeyFrame *pKeyFrame,
                                                                  double dist_threshold) {
    auto Rcw = pKeyFrame->GetRotation();
    auto tcw = pKeyFrame->GetTranslation();

    // keep track of the original point & convert to camera local frame
    std::vector<cv::Mat> vPointsLocal(vObjMapPoints.size());
    std::vector< std::pair<MapPoint*, double> > vCentroidDist(vObjMapPoints.size());

    cv::Mat centroid(3,1, CV_32F, 0.0f);

    for (int i=0; i < vObjMapPoints.size(); i++){
        vPointsLocal[i] = Rcw * vObjMapPoints[i]->GetWorldPos() + tcw;
        centroid = centroid + vPointsLocal[i];
    }

    centroid = centroid / (double)vObjMapPoints.size();

    for (int i=0; i < vObjMapPoints.size(); i++){
        vCentroidDist[i].first = vObjMapPoints[i];
        // TODO -- generalized dist calculation
        vCentroidDist[i].second = std::abs(vPointsLocal[i].at<float>(2) - centroid.at<float>(2)); // z axis only, todo: generalized this
    }

    std::sort(vCentroidDist.begin(), vCentroidDist.end(), [](const std::pair<MapPoint*, double> &a,
            const std::pair<MapPoint*, double> &b){
        return a.second < b.second;
    });

    int idx=1;

    for (; idx < vCentroidDist.size(); idx++){
      if ( (vCentroidDist[idx].second - vCentroidDist[idx-1].second) >  dist_threshold)
          break;
    }

    vFilteredMapPoints.resize(idx);

    for (int i=0; i< idx; i++){
        vFilteredMapPoints[i] = vCentroidDist[i].first;
    }
}

void PointCloudObjectInitializer::FilterMapPointsDistFromCentroidNormalized(
        const vector<ORB_SLAM2::MapPoint *> &vObjMapPoints, vector<ORB_SLAM2::MapPoint *> &vFilteredMapPoints,
        ORB_SLAM2::KeyFrame *pKeyFrame, double std_threshold) {

    auto Rcw = pKeyFrame->GetRotation();
    auto tcw = pKeyFrame->GetTranslation();

    // keep track of the original point & convert to camera local frame
    std::vector<cv::Mat> vPointsLocal(vObjMapPoints.size());
    std::vector< std::pair<MapPoint*, double> > vCentroidDist(vObjMapPoints.size());

    cv::Mat centroid(3,1, CV_32F, 0.0f);
    //static int filenum = 0;
    //std::ofstream myfile;
    //myfile.open("debug/points_" + std::to_string(filenum++) + ".csv");

    for (int i=0; i < vObjMapPoints.size(); i++){
        vPointsLocal[i] = Rcw * vObjMapPoints[i]->GetWorldPos() + tcw;
        centroid = centroid + vPointsLocal[i];
        //myfile << vPointsLocal[i].at<float>(0) << ','
        //        << vPointsLocal[i].at<float>(1) << ',' << vPointsLocal[i].at<float>(2) << '\n';
    }
    //myfile.close();


    centroid = centroid / (double)vObjMapPoints.size();

    double sigma_dist = 0, mu_dist = 0;

    for (int i=0; i < vObjMapPoints.size(); i++){
        vCentroidDist[i].first = vObjMapPoints[i];
        // TODO -- generalized dist calculation
        vCentroidDist[i].second = std::abs(vPointsLocal[i].at<float>(2) - centroid.at<float>(2)); // z axis only, todo: generalized this
        mu_dist += vCentroidDist[i].second;
    }

    mu_dist /= (double)vObjMapPoints.size();

    for (int i=0; i < vCentroidDist.size(); i++){
        auto d = vCentroidDist[i].second - mu_dist;
        sigma_dist += d * d;
    }

    sigma_dist = std::sqrt(sigma_dist / (double)vCentroidDist.size());
    // normalized Z-Score values in vCentroidDist
    for (int i=0; i < vCentroidDist.size(); i++){
        vCentroidDist[i].second = (vCentroidDist[i].second - mu_dist) / sigma_dist;
    }


    std::sort(vCentroidDist.begin(), vCentroidDist.end(), [](const std::pair<MapPoint*, double> &a,
                                                             const std::pair<MapPoint*, double> &b){
        return a.second < b.second;
    });

    // Compute std cache vector
    double std_acc=0, mu_acc=0; // linear time
    std::vector<double> std_acc_cache(vCentroidDist.size(), 0);

    for (int i=0; i < std_acc_cache.size(); i++){
        mu_acc += vCentroidDist[i].second;
        auto N = (double)(i+1);
        auto d = vCentroidDist[i].second - (mu_acc / N);
        std_acc += d*d;
        std_acc_cache[i] = std::sqrt(std_acc / N);
    }

    int idx=1;

    for (; idx < vCentroidDist.size(); idx++){
        auto diff = (std_acc_cache[idx] - std_acc_cache[idx-1]);
        assert(!std::isnan(diff));
        if (  diff >  std_threshold)
            break;
    }

    vFilteredMapPoints.resize(idx);

    for (int i=0; i< idx; i++){
        vFilteredMapPoints[i] = vCentroidDist[i].first;
    }

}

void PointCloudObjectInitializer::InitializeObjects(KeyFrame *pKeyframe) {

    SPDLOG_INFO("Preparing Init {}", pKeyframe->mnId);
    auto vPredictedObjects = pKeyframe->GetObjectPredictions();
    // Retrieve neighbor keyframes in covisibility graph
    std::vector<int> vAssociatedCount(vPredictedObjects.size(), 0); // use int for debuging
    int nn = 20;

    // ------------- Object Association -------------------
    const vector<KeyFrame *> vpNeighKFs = pKeyframe->GetBestCovisibilityKeyFrames(nn);

    SPDLOG_INFO("Associate {}", pKeyframe->mnId);
    std::vector< std::shared_ptr< std::vector<MapPoint*> > > vPredictionMPs(vPredictedObjects.size());
    // Preprocessing Measurements
    for (size_t i = 0; i < vPredictedObjects.size(); i++) {
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

        if (vObjMapPoints.size() < 10) {
            continue; // Too few points for calculation, TODO: Set as parameters
        }


        if (mbUseStatRemoveOutlier) {
            std::shared_ptr< std::vector<MapPoint *> > pvFilteredMapPoints = std::make_shared< std::vector<MapPoint *> >();

            if (mOutlierFilterType == 0)
                FilterMapPointsSOR(vObjMapPoints, *pvFilteredMapPoints, mbProject2d, pKeyframe);
            else if (mOutlierFilterType == 1)
                FilterMapPointsDistFromCentroid(vObjMapPoints, *pvFilteredMapPoints, pKeyframe, mOutlierFilterThreshold);
            else if (mOutlierFilterType == 2)
                FilterMapPointsDistFromCentroidNormalized(vObjMapPoints,
                        *pvFilteredMapPoints, pKeyframe, mOutlierFilterThreshold);
            else
                throw std::runtime_error("OUTLIER FILTER TYPE NOT IMPLEMENTED");

            vPredictionMPs[i] = pvFilteredMapPoints;
        }
        else{
            std::shared_ptr<std::vector<MapPoint*> > ptr(&vObjMapPoints);
            vPredictionMPs[i] = ptr;
        }

        // TODO -- Initialize 3d measurement for all predictions
    }

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
            && pMO->GetProjectedBoundingBox(pKeyframe, bb, mbAccociateCentroid)) {
                // Find label match with nearest center
                double min_dist = std::numeric_limits<double>::max();
                long int min_dist_idx = -1;
                // TODO -- Q? should bb box center limited inside image or actual? (could be outside)
                cv::Point2f bb_center = (bb.tl() + bb.br()) * 0.5;
                for (size_t i = 0; i < vPredictedObjects.size(); i++) {
                    if (vCovisKFAssociatedFound[i] || !vPredictionMPs[i] ){
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
                if ((min_dist_idx > -1) && (min_dist < mAssociateMax2DDist)){

                    vAssociatedCount[min_dist_idx]++;
                    assert(!vCovisKFAssociatedFound[min_dist_idx]);
                    vCovisKFAssociatedFound[min_dist_idx] = true;
                    //SPDLOG_INFO("Associating Observation! idx={} of KFID={}, to ObjID={}",
                    //        min_dist_idx,
                    //        (*vit_kf)->mnId,
                    //        pMO->mnId);

                    if(AssociateConstraintSatisfy(pKeyframe, pMO)){
                        pMO->AddObservation(pKeyframe, min_dist_idx);
                        pMO->AddObservations(*vPredictionMPs[min_dist_idx]);

                        pKeyframe->AddMapObject(pMO, min_dist_idx); // TODO -- Add map object measurement
                        pKeyframe->CountGoodMapObjectObservation();
                        mpMap->AddMapObject(pMO);

                        if(!pMO->IsReady())
                            pMO->SetReady();
                    }
                }
            }
        }
    }

    // ------------- Init objects where there is no association-------------------

    SPDLOG_DEBUG("Init Object for KF:  {}", pKeyframe->mnId);
    for (size_t i = 0; i < vPredictedObjects.size(); i++) {
        if (vAssociatedCount[i] > 0)
            continue;

        if (!vPredictionMPs[i])
            continue;

        auto &pred = *vPredictedObjects[i];
        // - Calculate Cuboid parameters
        auto vFilteredMapPoints = vPredictionMPs[i];
        auto inliers = PCLConverter::toPointCloud(*vFilteredMapPoints);
        auto cuboid = CuboidFromPointCloud(inliers);

        uint32_t color = (static_cast<uint32_t>(std::rand() % 255) << 16) |
                         (static_cast<uint32_t>(std::rand() % 255) << 8) |
                         (static_cast<uint32_t>(std::rand() % 255));

        for (auto &pMP : *vFilteredMapPoints) {
            pMP->SetPointColor(color); // For Debuging
        }

        // Create new MapObject -- pattern from LocalMapping.cc MapPoint!
        MapObject *pMO = new MapObject(cuboid, pred._label, pKeyframe, mpMap);
        pMO->AddObservation(pKeyframe, i);
        pMO->AddObservations(*vPredictionMPs[i]);
        pKeyframe->AddMapObject(pMO, i);
        mpMap->AddMapObject(pMO);
    }
}

void PointCloudObjectInitializer::InitializedObjectsWithGravity(ORB_SLAM2::KeyFrame *pKeyframe, const cv::Mat &g) {

    Eigen::Vector3f gNormalized = Converter::toVector3f(g / cv::norm(g));
    SPDLOG_INFO("Preparing Init {}", pKeyframe->mnId);
    auto vPredictedObjects = pKeyframe->GetObjectPredictions();
    // Retrieve neighbor keyframes in covisibility graph
    std::vector<int> vAssociatedCount(vPredictedObjects.size(), 0); // use int for debuging
    int nn = 20;

    // ------------- Object Association -------------------
    const vector<KeyFrame *> vpNeighKFs = pKeyframe->GetBestCovisibilityKeyFrames(nn);

    SPDLOG_INFO("Associate {}", pKeyframe->mnId);
    std::vector< std::shared_ptr< std::vector<MapPoint*> > > vPredictionMPs(vPredictedObjects.size());
    std::vector<Cuboid*> vPredictionCuboidEst(vPredictedObjects.size(), static_cast<Cuboid*>(NULL));
    // Preprocessing Measurements
    for (size_t i = 0; i < vPredictedObjects.size(); i++) {
        auto &pred = *vPredictedObjects[i];
        const auto box = pred.box();
        int bboxSizeThresh = min(pKeyframe->mnMaxX - pKeyframe->mnMinX, pKeyframe->mnMaxY - pKeyframe->mnMinY) / 10;

        if (box.width < bboxSizeThresh || box.height < bboxSizeThresh) {
            continue;
        }

        // if the keyframe does not have object then continue
        if (mbUseMask && pred._mask_type == PredictedObject::MASK_TYPE::NO_MASK)
            SPDLOG_WARN("Configure to use mask, but no mask provided");

        auto vObjMapPoints = mbUseMask && (pred._mask_type != PredictedObject::MASK_TYPE::NO_MASK) ?
                             pKeyframe->GetMapPointsInMask(box, pred._mask, pred._mask_type)
                                                                                                   : pKeyframe->GetMapPointsInBoundingBox(
                        box);


        if (vObjMapPoints.size() < 10) {
            continue; // Too few points for calculation, TODO: Set as parameters
        }


        if (mbUseStatRemoveOutlier) {
            std::shared_ptr<std::vector<MapPoint *> > pvFilteredMapPoints = std::make_shared<std::vector<MapPoint *> >();

            if (mOutlierFilterType == 0)
                FilterMapPointsSOR(vObjMapPoints, *pvFilteredMapPoints, mbProject2d, pKeyframe);
            else if (mOutlierFilterType == 1)
                FilterMapPointsDistFromCentroid(vObjMapPoints, *pvFilteredMapPoints, pKeyframe,
                                                mOutlierFilterThreshold);
            else if (mOutlierFilterType == 2)
                FilterMapPointsDistFromCentroidNormalized(vObjMapPoints,
                                                          *pvFilteredMapPoints, pKeyframe, mOutlierFilterThreshold);
            else
                throw std::runtime_error("OUTLIER FILTER TYPE NOT IMPLEMENTED");

            vPredictionMPs[i] = pvFilteredMapPoints;
        } else {
            std::shared_ptr<std::vector<MapPoint *> > ptr(&vObjMapPoints);
            vPredictionMPs[i] = ptr;
        }

        // Initial Cuboid HERE !

        // auto vFilteredMapPoints = vPredictionMPs[i];
        auto cuboid = CuboidFromPointCloudWithGravity(PCLConverter::toPointCloud(*vPredictionMPs[i]),
                                                      gNormalized);
        vPredictionCuboidEst[i] = cuboid;

    }

    {
        // Too lazy to write setter
        std::lock_guard<std::mutex> lock(pKeyframe->mMutexObject);
        pKeyframe->mvObjectPredictionCuboidEst = vPredictionCuboidEst;
    }

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
                && pMO->GetProjectedBoundingBox(pKeyframe, bb, mbAccociateCentroid)) {
                // Find label match with nearest center
                double min_dist = std::numeric_limits<double>::max();
                long int min_dist_idx = -1;
                // TODO -- Q? should bb box center limited inside image or actual? (could be outside)
                cv::Point2f bb_center = (bb.tl() + bb.br()) * 0.5;
                for (size_t i = 0; i < vPredictedObjects.size(); i++) {
                    if (vCovisKFAssociatedFound[i] || !vPredictionMPs[i] ){
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
                if ((min_dist_idx > -1) && (min_dist < mAssociateMax2DDist)){
                    vAssociatedCount[min_dist_idx]++;
                    assert(!vCovisKFAssociatedFound[min_dist_idx]);
                    vCovisKFAssociatedFound[min_dist_idx] = true;
                    //SPDLOG_INFO("Associating Observation! idx={} of KFID={}, to ObjID={}",
                    //        min_dist_idx,
                    //        (*vit_kf)->mnId,
                    //        pMO->mnId);

                    if(AssociateConstraintSatisfy(pKeyframe, pMO)){
                        pMO->AddObservation(pKeyframe, min_dist_idx);
                        pMO->AddObservations(*vPredictionMPs[min_dist_idx]);

                        pKeyframe->AddMapObject(pMO, min_dist_idx); // TODO -- Add map object measurement
                        pKeyframe->CountGoodMapObjectObservation();
                        mpMap->AddMapObject(pMO);

                        if(!pMO->IsReady())
                            pMO->SetReady();
                    }
                }
            }
        }
    }

    // ------------- Init objects where there is no association-------------------

    SPDLOG_DEBUG("Init Object for KF:  {}", pKeyframe->mnId);
    for (size_t i = 0; i < vPredictedObjects.size(); i++) {
        if (vAssociatedCount[i] > 0)
            continue;

        if (!vPredictionMPs[i])
            continue;

        // - Calculate Cuboid parameters
        // auto vFilteredMapPoints = vPredictionMPs[i];
        // auto inliers = PCLConverter::toPointCloud(*vFilteredMapPoints);
        // auto cuboid = CuboidFromPointCloudWithGravity(inliers, gNormalized);

        // Create new MapObject -- pattern from LocalMapping.cc MapPoint!
        MapObject *pMO = new MapObject(*vPredictionCuboidEst[i], vPredictedObjects[i]->_label, pKeyframe, mpMap);
        pMO->AddObservation(pKeyframe, i);
        pMO->AddObservations(*vPredictionMPs[i]);
        pKeyframe->AddMapObject(pMO, i);
        mpMap->AddMapObject(pMO);
    }
}

bool PointCloudObjectInitializer::AssociateConstraintSatisfy(ORB_SLAM2::KeyFrame* pKF,
                                                             ORB_SLAM2::MapObject* pMO) {

    auto pObjectLastKF = pMO->GetLatestKFObservation();
    if ((mAssociateConstraint == 0) || !pObjectLastKF)
        return true;


    bool ret = false;

    if (mAssociateConstraint & 0x01){
        // Temporal constraint
        double diff = pKF->mTimeStamp - pObjectLastKF->mTimeStamp;
        SPDLOG_DEBUG("[KF {} to {}] Time diff: {}", pKF->mnId, pObjectLastKF->mnId, diff);
        if (diff > mAssociateTimeDiff)
            ret |= true;
    }

    if (mAssociateConstraint & 0x02){
        // http://www.boris-belousov.net/2016/12/01/quat-dist/
        // cos(theta) = (trace(R1*R2^{t}) - 1) / 2

        // 1. Calculate estimate relative pose
        cv::Mat Rdiff;
        // 1.1 Naive solution: Base on world coordinate
        // Rdiff = pKF->GetRotation() * pObjectLastKF->GetRotation().t();
        // 1.2 Ideal solution, base on current tracking pose angle diff to object
        cv::Mat Two = pMO->GetPose();
        cv::Mat Tc1o = pKF->GetPoseInverse() * Two;
        cv::Mat Tc2o = pObjectLastKF->GetPoseInverse() * Two;
        Rdiff = Tc1o.rowRange(0,3).colRange(0,3) * Tc2o.rowRange(0,3).colRange(0,3).t();
        // Note for rotation matrix, either Rc?o or Roc? is okay! cuz we need to transpose it anyway
        double traceR = Rdiff.at<float>(0,0) +  Rdiff.at<float>(1,1) + Rdiff.at<float>(2,2);
        auto diff = std::acos((traceR - 1.0) / 2.0);

        if (std::isnan(diff)){
            ret |= true;
        }
        else{
            SPDLOG_DEBUG("[KF {} to {}] Angle diff: {}", pKF->mnId, pObjectLastKF->mnId, diff);
            if (diff > mAssociateAngleDiff)
                ret |= true;
        }
    }

    return ret;
}



}