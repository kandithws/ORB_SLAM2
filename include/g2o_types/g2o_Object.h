#pragma once

#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "utils/matrix_utils.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>
#include <algorithm>    // std::swap
#include "../Cuboid.h"

namespace g2o {

using namespace ORB_SLAM2;
using namespace ORB_SLAM2::utils;


class VertexCuboid : public BaseVertex<9, Cuboid>  // NOTE  this vertex stores object pose to world
{
public:
    inline VertexCuboid() = default;

    inline void setToOriginImpl() override { _estimate = Cuboid(); }

    inline void oplusImpl(const double* update_) override
    {
        Eigen::Map<const Eigen::Vector9d> update(update_);
        setEstimate(_estimate.expUpdate(update));
    }

    inline bool read(std::istream& is) override
    {
        Eigen::Vector9d est;
        for (int i = 0; i < 9; i++)
            is >> est[i];
        Cuboid cuboid;
        cuboid.fromMinimalVector(est);
        setEstimate(cuboid);
        return true;
    }

    inline bool write(std::ostream& os) const override
    {
        Eigen::Vector9d lv = _estimate.toMinimalVector();
        for (int i = 0; i < lv.rows(); i++) {
            os << lv[i] << " ";
        }
        return os.good();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// camera -object 3D error
class EdgeSE3Cuboid : public BaseBinaryEdge<9, Cuboid, VertexSE3Expmap, VertexCuboid> {
public:
    inline EdgeSE3Cuboid() = default;

    inline bool read(std::istream& is) override
    {
        return true;
    };

    inline bool write(std::ostream& os) const override
    {
        return os.good();
    };

    inline void computeError() override
    {
        const auto* SE3Vertex = dynamic_cast<const VertexSE3Expmap*>(_vertices[0]); //  world to camera pose
        const auto* cuboidVertex = dynamic_cast<const VertexCuboid*>(_vertices[1]); //  object pose to world

        SE3Quat cam_pose_Twc = SE3Vertex->estimate().inverse();
        const Cuboid& global_cube = cuboidVertex->estimate();
        Cuboid esti_global_cube = _measurement.transformFrom(cam_pose_Twc);
        _error = global_cube.minLogError(esti_global_cube);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// camera -object 2D projection error, rectangle difference, could also change to iou
class EdgeSE3CuboidProj : public BaseBinaryEdge<4, Eigen::Vector4d, VertexSE3Expmap, VertexCuboid> {
public:
    EdgeSE3CuboidProj(const Eigen::Matrix3d& K){
        Kalib = K;
    }

    inline bool read(std::istream& is) override
    {
        return true;
    };

    inline bool write(std::ostream& os) const override
    {
        return os.good();
    };

    inline void computeError() override
    {
        const auto* SE3Vertex = dynamic_cast<const VertexSE3Expmap*>(_vertices[0]);  //  world to camera pose
        const auto* cuboidVertex = dynamic_cast<const VertexCuboid*>(_vertices[1]);       //  object pose to world

        const SE3Quat& cam_pose_Tcw = SE3Vertex->estimate();
        const Cuboid& global_cube = cuboidVertex->estimate();

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

    inline bool read(std::istream& is) override
    {
        return true;
    };

    inline bool write(std::ostream& os) const override
    {
        return os.good();
    };

    inline void computeError() override
    {
        const auto* pointVertex = dynamic_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        const auto* cuboidVertex = dynamic_cast<const VertexCuboid*>(_vertices[1]);
        const auto& cuboid = cuboidVertex->estimate();
        const auto cuboid_pose_inv = cuboid.mPose.inverse().to_homogeneous_matrix();

        const auto  p_o = homo_to_real_coord_vec<double>(
                cuboid_pose_inv * real_to_homo_coord_vec<double>(pointVertex->estimate())
                        );
        // TODO -- Assume no measurement for now (use map point vertex as a measurement)
        const auto& dm = cuboid.mScale;

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

    inline bool read(std::istream& is) override
    {
        return true;
    };

    inline bool write(std::ostream& os) const override
    {
        return os.good();
    };

    inline void computeError() override
    {
        //const auto* pointVertex = dynamic_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        const auto* cuboidVertex = dynamic_cast<const VertexCuboid*>(_vertices[0]);
        const auto& cuboid = cuboidVertex->estimate();
        // const auto cuboid_pose_inv = cuboid.mPose.inverse().to_homogeneous_matrix();

        //const auto  p_o = homo_to_real_coord_vec<double>(
        //        cuboid_pose_inv * real_to_homo_coord_vec<double>(_measurement)
        //);

        const auto& dm = cuboid.mScale;

        _error = Eigen::Vector3d({
            std::max<double>(std::abs(_measurement[0]) - dm[0], 0.0),
            std::max<double>(std::abs(_measurement[1]) - dm[1], 0.0),
            std::max<double>(std::abs(_measurement[2]) - dm[2], 0.0)
        });
    }


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
