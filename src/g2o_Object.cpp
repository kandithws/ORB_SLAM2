//
// Created by kandithws on 8/12/2562.
//

#include <g2o_types/g2o_Object.h>


namespace g2o {

SE3Quat exptwist_norollpitch(const Eigen::Vector6d &update)
{
    Eigen::Vector3d omega;
    for (int i = 0; i < 3; i++)
        omega[i] = update[i];
    Eigen::Vector3d upsilon;
    for (int i = 0; i < 3; i++)
        upsilon[i] = update[i + 3];

    double theta = omega.norm();
    Eigen::Matrix3d Omega = skew(omega);

    Eigen::Matrix3d R;
    R << cos(omega(2)), -sin(omega(2)), 0,
            sin(omega(2)), cos(omega(2)), 0,
            0, 0, 1;

    Eigen::Matrix3d V;
    if (theta < 0.00001)
    {
        V = R;
    }
    else
    {
        Eigen::Matrix3d Omega2 = Omega * Omega;

        V = (Eigen::Matrix3d::Identity() + (1 - cos(theta)) / (theta * theta) * Omega + (theta - sin(theta)) / (pow(theta, 3)) * Omega2);
    }

    return SE3Quat(Quaterniond(R), V * upsilon);
}

VertexCuboid::VertexCuboid(bool update_rollpitch, bool update_scale) {
    update_rollpitch_ = update_rollpitch;
    update_scale_ = update_scale;
}

void VertexCuboid::oplusImpl(const double *update_){
    Eigen::Map<const Vector9d> update(update_);

    Cuboid newcube;

    // Pose update
    if (update_rollpitch_){
        newcube.mPose = _estimate.mPose * SE3Quat::exp(update.head<6>());
    }
    else{
        Eigen::Vector9d update2 = update;
        update2(0) = 0;
        update2(1) = 0;
        newcube.mPose = _estimate.mPose * exptwist_norollpitch(update2.head<6>());
    }

    if (update_scale_){
        // Original
        newcube.mScale = _estimate.mScale + update.tail<3>();
        // TODO -- apply some threshold logic

        // Logic 0; naive check!, if this update make < 0 do not update
        if ( (newcube.mScale(0) < epsillon) || (newcube.mScale(2) < epsillon) || (newcube.mScale(3) < epsillon)){
            newcube.mScale = _estimate.mScale;
        }
        std::cout << "[Current scale]=" << newcube.mScale(0) << ',' << newcube.mScale(1)
        << ',' << newcube.mScale(2)  << std::endl;



        // Logic update 1: Keep aspect ratio
        // double ratio = _estimate.mScale.sum();

        // double v_update_per_dim = cbrt(fabs(update.tail<3>().prod()) ); // cubic root


        //if (update.tail<3>().sum() < 0){
        //    v_update_per_dim *= -1.0;
        // }

        //Eigen::Vector3d v_update = (_estimate.mScale / ratio) * v_update_per_dim;

        //newcube.mScale = _estimate.mScale + v_update;
        //std::cout << "[Scale] ratio=" << ratio << ", v_update_per_dim=" << v_update_per_dim;
        // std::cout << "[Scale] new update=" << v_update << ", org update="<< update.tail<3>() << std::endl;

    }
    else{
        // Fix scale from previous
        newcube.mScale = _estimate.mScale;
    }


    setEstimate(newcube);
}

};