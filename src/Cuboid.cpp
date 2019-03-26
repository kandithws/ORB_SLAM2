//
// Created by kandithws on 26/3/2562.
//

#include "Cuboid.h"

namespace ORB_SLAM2 {
using namespace g2o;

void Cuboid::fromMinimalVector(const Eigen::Vector9d &v) {
    Eigen::Quaterniond posequat = utils::zyx_euler_to_quat(v(3), v(4), v(5));
    mPose = SE3Quat(posequat, v.head<3>());
    mScale = v.tail<3>();
}

// xyz quaternion, half_scale
void Cuboid::fromVector(const Eigen::Vector10d &v) {
    mPose.fromVector(v.head<7>());
    mScale = v.tail<3>();
}

Cuboid Cuboid::expUpdate(const Eigen::Vector9d &update) {
    Cuboid res;
    res.mPose = this->mPose * SE3Quat::exp(update.head<6>()); // NOTE bug before. switch position
    res.mScale = this->mScale + update.tail<3>();
    return res;
}

Eigen::Vector9d Cuboid::logCubeError(const Cuboid &other) const {
    Eigen::Vector9d errVec;
    SE3Quat poseDiff = other.mPose.inverse() * this->mPose;
    errVec.head<6>() = poseDiff.log();   //treat as se3 log error. could also just use yaw error
    errVec.tail<3>() = this->mScale - other.mScale;
    return errVec;
}


Eigen::Vector9d Cuboid::minLogError(const Cuboid &other, bool printDetails) const {
    bool whether_rotate_cubes = true;  // whether rotate cube to find smallest error
    if (!whether_rotate_cubes)
        return logCubeError(other);

// NOTE rotating cuboid... since we cannot determine the front face consistently,
// different front faces indicate different yaw, scale representation.
// need to rotate all 360 degrees (global cube might be quite different from local cube)
// this requires the sequential object insertion. In this case, object yaw practically should not change much.
// If we observe a jump, we can use code here to adjust the yaw.
    Eigen::Vector4d rotate_errors_norm;
    Eigen::Vector4d rotate_angles(-1, 0, 1, 2); // rotate -90 0 90 180
    Eigen::Matrix<double, 9, 4> rotate_errors;
    for (int i = 0; i < rotate_errors_norm.rows(); i++) {
        Cuboid rotated_cuboid = other.rotateCuboid(rotate_angles(i) * M_PI_2);  // rotate new cuboids
        Eigen::Vector9d cuboid_error = this->logCubeError(rotated_cuboid);
        rotate_errors_norm(i) = cuboid_error.norm();
        rotate_errors.col(i) = cuboid_error;
    }
    int min_label;
    rotate_errors_norm.minCoeff(&min_label);
    if (printDetails)
        if (min_label != 1)
            std::cout << "Rotate cube   " << min_label << std::endl;
    return rotate_errors.col(min_label);
}

Cuboid Cuboid::rotateCuboid(double yaw_angle) const // to deal with different front surface of cuboids
{
    Cuboid res;
    // change yaw to rotation.
    SE3Quat rot(Eigen::Quaterniond(cos(yaw_angle *
    0.5), 0, 0, sin(yaw_angle * 0.5)), Eigen::Vector3d(0, 0, 0));
    res.mPose = this->mPose * rot;
    res.mScale = this->mScale;
    if ((yaw_angle == M_PI / 2.0) || (yaw_angle == -M_PI / 2.0) || (yaw_angle == 3 * M_PI / 2.0))
        std::swap(res.mScale(0), res.mScale(1));

    return res;
}

Cuboid Cuboid::transformFrom(const SE3Quat& Twc) const
{
    Cuboid res;
    res.mPose = Twc * this->mPose;
    res.mScale = this->mScale;
    return res;
}

Cuboid Cuboid::transformTo(const SE3Quat& Twc) const
{
    Cuboid res;
    res.mPose = Twc.inverse() * this->mPose;
    res.mScale = this->mScale;
    return res;
}


Eigen::Vector9d Cuboid::toMinimalVector() const
{
    Eigen::Vector9d v;
    v.head<6>() = mPose.toXYZPRYVector();
    v.tail<3>() = mScale;
    return v;
}

// xyz quaternion, half_scale
Eigen::Vector10d Cuboid::toVector() const
{
    Eigen::Vector10d v;
    v.head<7>() = mPose.toVector();
    v.tail<3>() = mScale;
    return v;
}

Eigen::Matrix4d Cuboid::similarityTransform() const
{
    Eigen::Matrix4d res = mPose.to_homogeneous_matrix();
    Eigen::Matrix3d scale_mat = mScale.asDiagonal();
    res.topLeftCorner<3, 3>() = res.topLeftCorner<3, 3>() * scale_mat;
    return res;
}

// 8 corners 3*8 matrix, each row is x y z
Eigen::Matrix3Xd Cuboid::compute3D_BoxCorner() const
{
    Eigen::Matrix3Xd corners_body;
    corners_body.resize(3, 8);
    corners_body << 1, 1, -1, -1, 1, 1, -1, -1,
            1, -1, -1, 1, 1, -1, -1, 1,
            -1, -1, -1, -1, 1, 1, 1, 1;
    Eigen::Matrix3Xd corners_world = utils::homo_to_real_coord<double>(
            similarityTransform() * utils::real_to_homo_coord<double>(corners_body));
    return corners_world;
}

// get rectangles after projection  [topleft, bottomright]
Eigen::Vector4d Cuboid::projectOntoImageRect(const SE3Quat& campose_cw, const Eigen::Matrix3d& Kalib) const
{
    Eigen::Matrix3Xd corners_3d_world = compute3D_BoxCorner();
    Eigen::Matrix2Xd corner_2d = utils::homo_to_real_coord<double>(Kalib * utils::homo_to_real_coord<double>(
            campose_cw.to_homogeneous_matrix() * utils::real_to_homo_coord<double>(corners_3d_world)));
    Eigen::Vector2d bottomright = corner_2d.rowwise().maxCoeff(); // x y
    Eigen::Vector2d topleft = corner_2d.rowwise().minCoeff();
    return {topleft(0), topleft(1), bottomright(0), bottomright(1)};
}

// get rectangles after projection  [center, width, height]
Eigen::Vector4d Cuboid::projectOntoImageBbox(const SE3Quat& campose_cw, const Eigen::Matrix3d& Kalib) const
{
    Eigen::Vector4d rect_project = projectOntoImageRect(campose_cw, Kalib);  // top_left, bottom_right  x1 y1 x2 y2
    Eigen::Vector2d rect_center = (rect_project.tail<2>() + rect_project.head<2>()) / 2;
    Eigen::Vector2d widthheight = rect_project.tail<2>() - rect_project.head<2>();
    return {rect_center(0), rect_center(1), widthheight(0), widthheight(1)};
}

}