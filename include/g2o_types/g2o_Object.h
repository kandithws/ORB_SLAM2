#pragma once

#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "utils/matrix_utils.h"
#include "utils/vector_utils.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>
#include <algorithm>    // std::swap
#include "../Cuboid.h"
#include <utils/Config.h>

namespace g2o {

using namespace ORB_SLAM2;
using namespace ORB_SLAM2::utils;


class VertexCuboid : public BaseVertex<9, Cuboid>  // NOTE  this vertex stores object pose to world
{
  public:
    //VertexCuboid() = default;
    VertexCuboid(bool update_rollpitch=true, bool update_scale=true);

    inline void setToOriginImpl() override { _estimate = Cuboid(); }

//    inline void oplusImpl(const double *update_) override {
//        Eigen::Map<const Eigen::Vector9d> update(update_);
//        setEstimate(_estimate.expUpdate(update));
//    }
    void oplusImpl(const double *update_);

    inline bool read(std::istream &is) override {
        Eigen::Vector9d est;
        for (int i = 0; i < 9; i++)
            is >> est[i];
        Cuboid cuboid;
        cuboid.fromMinimalVector(est);
        setEstimate(cuboid);
        return true;
    }

    inline bool write(std::ostream &os) const override {
        Eigen::Vector9d lv = _estimate.toMinimalVector();
        for (int i = 0; i < lv.rows(); i++) {
            os << lv[i] << " ";
        }
        return os.good();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    bool update_rollpitch_ = true;
    bool update_scale_=true;
    double epsillon = 1e-2;
};

// camera -object 3D error
class EdgeSE3Cuboid : public BaseBinaryEdge<9, Cuboid, VertexSE3Expmap, VertexCuboid> {
  public:
    inline EdgeSE3Cuboid() = default;

    inline bool read(std::istream &is) override {
        return true;
    };

    inline bool write(std::ostream &os) const override {
        return os.good();
    };

    inline void computeError() override {
        const auto *SE3Vertex = dynamic_cast<const VertexSE3Expmap *>(_vertices[0]); //  world to camera pose
        const auto *cuboidVertex = dynamic_cast<const VertexCuboid *>(_vertices[1]); //  object pose to world

        SE3Quat cam_pose_Twc = SE3Vertex->estimate().inverse();
        const Cuboid &global_cube = cuboidVertex->estimate();
        Cuboid esti_global_cube = _measurement.transformFrom(cam_pose_Twc);
        _error = global_cube.minLogError(esti_global_cube);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// camera -object 2D projection error, rectangle difference, could also change to iou
class EdgeSE3CuboidProj : public BaseBinaryEdge<4, Eigen::Vector4d, VertexSE3Expmap, VertexCuboid> {
  public:
    EdgeSE3CuboidProj(const Eigen::Matrix3d &K) {
        Kalib = K;
    }

    inline bool read(std::istream &is) override {
        return true;
    };

    inline bool write(std::ostream &os) const override {
        return os.good();
    };

    inline void computeError() override {
        const auto *SE3Vertex = dynamic_cast<const VertexSE3Expmap *>(_vertices[0]);  //  world to camera pose
        const auto *cuboidVertex = dynamic_cast<const VertexCuboid *>(_vertices[1]);       //  object pose to world

        const SE3Quat &cam_pose_Tcw = SE3Vertex->estimate();
        const Cuboid &global_cube = cuboidVertex->estimate();

        Eigen::Vector4d rect_project = global_cube.projectOntoImageBbox(cam_pose_Tcw, Kalib); // center, width, height

        _error = rect_project - _measurement;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
    Eigen::Matrix3d Kalib;
};

// MapPoint which should be included in cuboid
class EdgeCuboidMapPoint : public BaseBinaryEdge<3, Eigen::Vector3d, VertexSBAPointXYZ, VertexCuboid> {
  public:
    EdgeCuboidMapPoint() = default;

    inline bool read(std::istream &is) override {
        return true;
    };

    inline bool write(std::ostream &os) const override {
        return os.good();
    };

    inline void computeError() override {
        const auto *pointVertex = dynamic_cast<const VertexSBAPointXYZ *>(_vertices[0]);
        const auto *cuboidVertex = dynamic_cast<const VertexCuboid *>(_vertices[1]);
        const auto &cuboid = cuboidVertex->estimate();
        const auto cuboid_pose_inv = cuboid.mPose.inverse().to_homogeneous_matrix();

        const auto p_o = homo_to_real_coord_vec<double>(
                cuboid_pose_inv * real_to_homo_coord_vec<double>(pointVertex->estimate())
        );
        // TODO -- Assume no measurement for now (use map point vertex as a measurement)
        const auto &dm = cuboid.mScale;

        _error = Eigen::Vector3d({
                                         std::max<double>(std::abs(p_o[0]) - dm[0], 0.0),
                                         std::max<double>(std::abs(p_o[1]) - dm[1], 0.0),
                                         std::max<double>(std::abs(p_o[2]) - dm[2], 0.0)
                                 });
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

// Not optimizing the map points, just take the value only
class EdgeCuboidMapPointUnary : public BaseUnaryEdge<3, Eigen::Vector3d, VertexCuboid> {
  public:
    EdgeCuboidMapPointUnary() = default;

    inline bool read(std::istream &is) override {
        return true;
    };

    inline bool write(std::ostream &os) const override {
        return os.good();
    };

    inline void computeError() override {
        //const auto* pointVertex = dynamic_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        const auto *cuboidVertex = dynamic_cast<const VertexCuboid *>(_vertices[0]);
        const auto &cuboid = cuboidVertex->estimate();
        // const auto cuboid_pose_inv = cuboid.mPose.inverse().to_homogeneous_matrix();

        //const auto  p_o = homo_to_real_coord_vec<double>(
        //        cuboid_pose_inv * real_to_homo_coord_vec<double>(_measurement)
        //);

        const auto &dm = cuboid.mScale;

        _error = Eigen::Vector3d({
            std::max<double>(std::abs(_measurement[0]) - dm[0], 0.0),
            std::max<double>(std::abs(_measurement[1]) - dm[1], 0.0),
            std::max<double>(std::abs(_measurement[2]) - dm[2], 0.0)
                                 });
    }


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


class EdgeCuboidMapPointUnaryBatch : public BaseUnaryEdge<3, Eigen::Vector3d, VertexCuboid> {
  public:
    EdgeCuboidMapPointUnaryBatch(const ORB_SLAM2::utils::eigen_aligned_vector<Eigen::Vector3d>& vMPWorldPosHomo,
            const double& max_margin_ratio=1.0): mMaxMarginRatio(max_margin_ratio)
    {
        mvMPWorldPos = vMPWorldPosHomo;
    }

    inline bool read(std::istream &is) override {
        return true;
    };

    inline bool write(std::ostream &os) const override {
        return os.good();
    };

//    inline void computeError() override {
//        //const auto* pointVertex = dynamic_cast<const VertexSBAPointXYZ*>(_vertices[0]);
//        const auto *cuboidVertex = dynamic_cast<const VertexCuboid *>(_vertices[0]);
//        const auto &cuboid = cuboidVertex->estimate();
//        _error.setZero();
//
//        const auto cuboid_pose_inv = cuboid.mPose.inverse().to_homogeneous_matrix();
//        const auto &dm = cuboid.mScale;
//        for (const auto& point_pose : mvMPWorldPos){
//            Eigen::Vector4d point_pose_local =  cuboid_pose_inv * point_pose;
//
//            _error += Eigen::Vector3d({
//                                             std::max<double>(std::abs(point_pose_local[0]) - dm[0], 0.0),
//                                             std::max<double>(std::abs(point_pose_local[1]) - dm[1], 0.0),
//                                             std::max<double>(std::abs(point_pose_local[2]) - dm[2], 0.0)
//                                     });
//        }
//
//        // Normalizing Error
//
//        _error = _error / (double)mvMPWorldPos.size();
//    }


    inline void computeError() override {
        //const auto* pointVertex = dynamic_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        const auto *cuboidVertex = dynamic_cast<const VertexCuboid *>(_vertices[0]);
        const auto &cuboid = cuboidVertex->estimate();
        _error.setZero();
        Eigen::Vector3d points_error;
        points_error.setZero();

        for (const auto& point_pose : mvMPWorldPos){

            points_error += cuboid.computePointBoundaryError(point_pose, mMaxMarginRatio).cwiseAbs();
        }

        // Normalizing Error
        if(mvMPWorldPos.size() > 0)
            points_error = points_error / (double)mvMPWorldPos.size();

        _error = points_error;
    }

    ORB_SLAM2::utils::eigen_aligned_vector<Eigen::Vector3d> mvMPWorldPos;

    const double mMaxMarginRatio;


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class EdgeCuboidGravityConstraint : public BaseUnaryEdge<3, Eigen::Vector3d, VertexCuboid>{
  public:
    EdgeCuboidGravityConstraint(){
        information() = Eigen::Matrix3d::Identity();
    };

    inline bool read(std::istream &is) override {
        return true;
    };

    inline bool write(std::ostream &os) const override {
        return os.good();
    };

    inline void computeError() override {
        const auto *cuboidVertex = dynamic_cast<const VertexCuboid *>(_vertices[0]);
        const auto &cuboid = cuboidVertex->estimate();

        Eigen::Vector3d z_rot_axis = cuboid.getRotation().toRotationMatrix().col(2);

        // Aligned with "Inverse unit gravity vector, so +"


        _error = z_rot_axis + _measurement;
        // _error = z_rot_axis.dot(_measurement) / z_rot_axis.norm();
        // double e =
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

  class EdgeCuboidGravityConstraint2 : public BaseUnaryEdge<1, double, VertexCuboid>{
  public:
    EdgeCuboidGravityConstraint2(const Eigen::Vector3d& gNormalized, double info_scale=1.0){
        information()[0] = info_scale;

        _g_normalized_inverse = -gNormalized;
    };

    inline bool read(std::istream &is) override {
        return true;
    };

    inline bool write(std::ostream &os) const override {
        return os.good();
    };

    inline void computeError() override {
        const auto *cuboidVertex = dynamic_cast<const VertexCuboid *>(_vertices[0]);
        const auto &cuboid = cuboidVertex->estimate();

        Eigen::Vector3d z_rot_axis = cuboid.getRotation().toRotationMatrix().col(2);

        // Aligned with "Inverse unit gravity vector, so +"


        // _error = z_rot_axis + _measurement;
        // Cosine distance
        _error[0] = 1.0 - z_rot_axis.dot(_g_normalized_inverse);
        // double e =
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:

    Eigen::Vector3d _g_normalized_inverse;
};

}
