//
// Created by kandithws on 24/3/2562.
//

#ifndef ORB_SLAM2_CUBOID_H
#define ORB_SLAM2_CUBOID_H

#include <pcl/pcl_base.h>
#include <mutex>
#include <cv.h>
#include "Thirdparty/g2o/g2o/types/se3quat.h"
#include "utils/matrix_utils.h"

namespace Eigen {
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;
typedef Eigen::Matrix<double, 3, 8> Matrix38d;
typedef Eigen::Matrix<double, 10, 1> Vector10d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

}

// Credit: https://github.com/kyu-sz/CubeSLAM/
// Internal Cuboid Representation
namespace ORB_SLAM2 {
using namespace g2o;
class Cuboid {
  public:
    Cuboid() { mScale.setZero(); }
    Cuboid(const Cuboid& cuboid){
        mPose = cuboid.mPose;
        mScale = cuboid.mScale;
    }
    // xyz roll pitch yaw half_scale
    void fromMinimalVector(const Eigen::Vector9d& v);

    // xyz quaternion, half_scale
    void fromVector(const Eigen::Vector10d& v);

    inline const Eigen::Vector3d& getTranslation() const { return mPose.translation(); }
    inline const Eigen::Quaterniond& getRotation() const { return mPose.rotation(); }
    inline void setTranslation(const Eigen::Vector3d& t_) { mPose.setTranslation(t_); }
    inline void setRotation(const Eigen::Quaterniond& r_) { mPose.setRotation(r_); }
    inline void setRotation(const Eigen::Matrix3d& R) { mPose.setRotation(Eigen::Quaterniond(R)); }
    inline void setScale(const Eigen::Vector3d& scale_) { mScale = scale_; }
    inline void setScale(float length, float width, float height)
    {
        mScale[0] = length;
        mScale[1] = width;
        mScale[2] = height;
    }

    // apply update to current cuboid. exponential map
    Cuboid expUpdate(const Eigen::Vector9d& update);

    // actual error between two cuboids.
    Eigen::Vector9d logCubeError(const Cuboid& other) const;

    // function called by g2o.
    Eigen::Vector9d minLogError(const Cuboid& other, bool printDetails = false) const;

    // change front face by rotate along current body z axis. another way of representing cuboid. representing same cuboid (IOU always 1)
    Cuboid rotateCuboid(double yaw_angle) const; // to deal with different front surface of cuboids

    // transform a local cuboid to global cuboid  Twc is camera pose. from camera to world
    Cuboid transformFrom(const SE3Quat& Twc) const;

    // transform a global cuboid to local cuboid  Twc is camera pose. from camera to world
    Cuboid transformTo(const SE3Quat& Twc) const;

    Eigen::Vector3d computePointBoundaryError(const Vector3d &world_point, const double& max_outside_margin_ratio) const;

    // bool isOutlierPoint(const Vector3d &world_point, const double& max_outside_margin_ratio);

    // xyz roll pitch yaw half_scale
    Eigen::Vector9d toMinimalVector() const;

    // xyz quaternion, half_scale
    Eigen::Vector10d toVector() const;

    Eigen::Matrix4d similarityTransform() const;

    // 8 corners 3*8 matrix, each row is x y z
    Eigen::Matrix3Xd compute3D_BoxCorner() const;

    // get rectangles after projection  [topleft, bottomright]
    Eigen::Vector4d projectOntoImageRect(const SE3Quat& campose_cw, const Eigen::Matrix3d& Kalib) const;

    // get rectangles after projection  [center, width, height]
    Eigen::Vector4d projectOntoImageBbox(const SE3Quat& campose_cw, const Eigen::Matrix3d& Kalib) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:
    SE3Quat mPose;  // 6 dof for object, object to world by default
    Eigen::Vector3d mScale; // [length, width, height]  half!
};
}

#endif //ORB_SLAM2_CUBOID_H
