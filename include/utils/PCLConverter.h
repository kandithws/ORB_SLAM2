//
// Created by kandithws on 6/3/2562.
//

#ifndef ORB_SLAM2_PCLCONVERTER_H
#define ORB_SLAM2_PCLCONVERTER_H

#include "../MapPoint.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ORB_SLAM2 {
class PCLConverter{
  public:
    typedef pcl::PointXYZRGBL PCLPointT;
    static pcl::PointCloud<PCLPointT>::Ptr toPointCloud(const std::vector<MapPoint*>& vMp);

    template <typename T>
    static void filterVector(const std::vector<int>& vIndices, const std::vector<T>& vIn, std::vector<T>& vOut){
        vOut.reserve(vIndices.size());
        for (const int &idx : vIndices){
            vOut.push_back(vIn[idx]);
        }
    }

    static void makeAffineTf(float x, float y, float z, float r, float p, float yaw, Eigen::Affine3f &tf) {
        Eigen::Vector3f t;
        t << x,y,z;
        Eigen::AngleAxisf rollAngle(r, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf pitchAngle(p, Eigen::Vector3f::UnitY());
        Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
        Eigen::Matrix4f tf_mat(Eigen::Matrix4f::Identity());
        tf_mat.block<3,3>(0,0) = q.toRotationMatrix();
        tf_mat.block<3,1>(0,3) = t;
        tf.matrix() = tf_mat;
    }

    static void makeAffineTf(const cv::Mat &mat, Eigen::Affine3f &tf){
        Eigen::Matrix3f R;
        R << mat.at<float>(0,0), mat.at<float>(0,1), mat.at<float>(0,2),
                mat.at<float>(1,0), mat.at<float>(1,1), mat.at<float>(1,2),
                mat.at<float>(2,0), mat.at<float>(2,1), mat.at<float>(2,2);

        Eigen::Vector3f t;
        t << mat.at<float>(0,3), mat.at<float>(1,3), mat.at<float>(2,3);

        Eigen::Matrix4f tf_mat(Eigen::Matrix4f::Identity());
        tf_mat.block<3,3>(0,0) = R;
        tf_mat.block<3,1>(0,3) = t;
        tf.matrix() = tf_mat;
    }

};
}
#endif //ORB_SLAM2_PCLCONVERTER_H
