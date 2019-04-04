//
// Created by kandithws on 23/1/2562.
//

#include <pcl_slam_visualizer/pcl_slam_visualizer.h>


namespace pcl{
namespace visualization {
PCLSLAMVisualizer::PCLSLAMVisualizer(const std::string &name, const bool create_interactor) :
PCLVisualizer(name, create_interactor) {}


bool PCLSLAMVisualizer::updateCube(const Eigen::Vector3f &t, const Eigen::Quaternionf &q, const double &sx,
                                   const double &sy, const double &sz, const std::string &id, int viewport) {
    if (!contains(id)){
        return (false);
    }

    auto shape_actor_map = getShapeActorMap();
    ShapeActorMap::iterator am_it = shape_actor_map->find(id);

    if (am_it == shape_actor_map->end ()){
        return false;
    }

    vtkSmartPointer<vtkLODActor> actor = vtkLODActor::SafeDownCast (am_it->second);
    vtkSmartPointer<vtkDataSet> data = createCube(t,q,sx,sy,sz);
    //createActorFromVTKDataSet (data, actor);
    // Set new cube to the exsisting actor, PCLVisualizer::createActorFromVTKDataSet

    vtkSmartPointer<vtkDataArray> scalars = data->GetPointData ()->GetScalars ();
    double minmax[2];
    if (scalars)
        scalars->GetRange (minmax);

    vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();

    mapper->SetInputData(data); // TODO: Check VTK version

    if (scalars)
        mapper->SetScalarRange (minmax);
    mapper->SetScalarModeToUsePointData ();
    mapper->ScalarVisibilityOn ();
    actor->SetNumberOfCloudPoints (data->GetNumberOfPoints () / 10);
    actor->GetProperty ()->SetInterpolationToFlat ();
    actor->SetMapper (mapper);
    actor->Modified();
    return true;
}

// TODO -- fix below impl
/*
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

    if (am_it == shape_actor_map->end ()){
        std::cout << "[UPDATE ERROR!!!]" << std::endl;
        return false;
    }
    std::cout << "[UPDATE 2]" << std::endl;


    vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second.GetPointer());
    // vtkActor* actor = vtkActor::SafeDownCast (am_it->second.GetPointer());
    // auto actor = dynamic_cast<vtkLODActor*>(am_it->second.GetPointer());
    if (!actor)
        return (false);

    std::cout << "[UPDATE 2.5]" << std::endl;
#if VTK_MAJOR_VERSION < 6
    vtkAlgorithm *algo = actor->GetMapper ()->GetInput ()->GetProducerPort ()->GetProducer ();
#else
    vtkAlgorithm *algo = actor->GetMapper ()->GetInputAlgorithm ();
#endif
    if(!algo){
        std::cout << "[ALGO ERROR!!!]" << std::endl;
        return false;
    }

    auto poly_algo = vtkPolyDataAlgorithm::SafeDownCast(algo);

    if(!poly_algo){
        std::cout << "[POLY ALGO ERROR!!!]" << std::endl;
        return false;
    }

    vtkCubeSource *src = vtkCubeSource::SafeDownCast (poly_algo);
    //auto src = dynamic_cast<vtkCubeSource*>(algo);

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
 */

}
}

