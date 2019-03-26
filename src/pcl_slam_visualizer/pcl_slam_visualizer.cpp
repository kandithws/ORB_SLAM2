//
// Created by kandithws on 23/1/2562.
//

#include <pcl_slam_visualizer/pcl_slam_visualizer.h>


namespace pcl{
namespace visualization {
PCLSLAMVisualizer::PCLSLAMVisualizer(const std::string &name, const bool create_interactor) :
PCLVisualizer(name, create_interactor) {}

bool PCLSLAMVisualizer::updateCube(const Eigen::Vector3f &t, const Eigen::Quaternionf &q,
                                   const double &sx, const double &sy, const double &sz
                                   , const std::string &id, int viewport) {

    std::cout << "[UPDATE 0]" << std::endl;
    if (!contains(id)){
        return (false);
    }
    std::cout << "[UPDATE 1]" << std::endl;

    auto shape_actor_map = getShapeActorMap();
    ShapeActorMap::iterator am_it = shape_actor_map->find(id);

    std::cout << "[UPDATE 2]" << std::endl;

    vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second);
    if (!actor)
        return (false);

#if VTK_MAJOR_VERSION < 6
    vtkAlgorithm *algo = actor->GetMapper ()->GetInput ()->GetProducerPort ()->GetProducer ();
#else
    vtkAlgorithm *algo = actor->GetMapper ()->GetInputAlgorithm ();
#endif
    vtkCubeSource *src = vtkCubeSource::SafeDownCast (algo);
    if (!src)
        return (false);

    std::cout << "[UPDATE 3]" << std::endl;
    src->SetXLength(sx);
    src->SetYLength(sy);
    src->SetZLength(sz);

    src->Update();

    std::cout << "[UPDATE 4]" << std::endl;

    vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New ();
    Eigen::Matrix4f tf_mat(Eigen::Matrix4f::Identity());
    tf_mat.block<3,3>(0,0) = q.toRotationMatrix();
    tf_mat.block<3,1>(0,3) = t;
    std::cout << "[UPDATE 5]" << std::endl;
    convertToVtkMatrix(tf_mat, matrix);
    std::cout << "[UPDATE 6]" << std::endl;
    actor->SetUserMatrix(matrix);
    std::cout << "[UPDATE 7]" << std::endl;
    actor->Modified();
    std::cout << "[UPDATE 8]" << std::endl;
    //convertToVtkMatrix();
    //vtkSmartPointer<vtkDataSet> data = createCube(coefficients);
    return true;
}

}
}

