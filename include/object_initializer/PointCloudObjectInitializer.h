#ifndef ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_H
#define ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_H

//#define ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_DEBUG

#include <object_initializer/IObjectInitializer.h>
#include <spdlog/spdlog.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "utils/PCLConverter.h"
#include "Cuboid.h"
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <utils/time.h>
#include <pcl/filters/project_inliers.h>

namespace ORB_SLAM2{
class Cuboid;

class PointCloudObjectInitializer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointCloudObjectInitializer(Map* pMap);
    void InitializeObjects(KeyFrame* pKeyframe);
    void InitializedObjectsWithGravity(KeyFrame* pKeyframe, const cv::Mat& g);
    //void InitializeObjectsWithGravity(KeyFrame pKeyFrame, Map* pMap, const cv::Mat &gNormalized);
#ifdef ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_DEBUG
    pcl::PCDWriter mCloudDebugWriter;
#endif
  private:
    Map* mpMap;
    bool mbProject2d;
    bool mbUseMask;
    bool mbUseStatRemoveOutlier;
    bool mbAccociateCentroid;
    int mOutlierFilterType;
    double mOutlierFilterThreshold;
    uint8_t mAssociateConstraint;
    double mAssociateTimeDiff;
    double mAssociateAngleDiff;

    Eigen::Matrix3f mMatrixRotatePitch90;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> mCloudSORFilter;
    pcl::ProjectInliers<pcl::PointXYZRGB> mProj;
    inline double Point2DDistance(const cv::Point2f& p1, const cv::Point2f& p2){
        return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    Cuboid CuboidFromPointCloud(pcl::PointCloud<PCLConverter::PCLPointT>::Ptr cloud);
    Cuboid* CuboidFromPointCloudWithGravity(
            pcl::PointCloud<PCLConverter::PCLPointT>::Ptr cloud,
            const Eigen::Vector3f& gNormalized);

    void FilterMapPointsSOR(const std::vector<MapPoint *> &vObjMapPoints,
            std::vector<MapPoint *> &vFilteredMapPoints, bool bProject2D = false, KeyFrame* pKeyFrame= static_cast<KeyFrame*>(NULL));

    void FilterMapPointsDistFromCentroid(const std::vector<MapPoint *> &vObjMapPoints,
            std::vector<MapPoint *> &vFilteredMapPoints, KeyFrame* pKeyFrame, double dist_threshold=0.2);

    void FilterMapPointsDistFromCentroidNormalized(const std::vector<MapPoint *> &vObjMapPoints,
                                                   std::vector<MapPoint *> &vFilteredMapPoints,
                                                   KeyFrame* pKeyFrame, double std_threshold=0.2);

    bool AssociateConstraintSatisfy(KeyFrame* pKF, MapObject* pMO);
};
}


#endif //ORB_SLAM2_BASEOBJECTINITIALIZER_H