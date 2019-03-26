//
// Created by kandithws on 23/1/2562.
//

#ifndef CUSTOM_PCL_VIEWER_PCLSLAMVISUALIZER_H
#define CUSTOM_PCL_VIEWER_PCLSLAMVISUALIZER_H
// TODO -- fixbug undefiend reference in customized_pcl_visualizer.h
//#include <pcl_slam_visualizer/customized_pcl_visualizer/customized_pcl_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkCubeSource.h>
namespace pcl {
namespace visualization{
class PCL_EXPORTS PCLSLAMVisualizer : public PCLVisualizer {
  public:
    PCLSLAMVisualizer(const std::string &name = "", const bool create_interactor = true);

    bool updateCube(const Eigen::Vector3f &t, const Eigen::Quaternionf &q,
            const double &sx, const double &sy, const double &sz,
            const std::string &id="cube", int viewport=0);
  // TODO -- add public methods to add camera/keyframes/trajectory etc ...
    // getRendererCollection() to add new type of actor! i.e. camera!
};
}
}

#endif //CUSTOM_PCL_VIEWER_PCLSLAMVISUALIZER_H
