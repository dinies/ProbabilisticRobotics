#include "ad_geometry.h"
#include <iostream>
#include <Eigen/StdVector>
#include "ad_multivariate.h"
#include "ad_geometry.h"

using namespace std;
using namespace AD;
using namespace Eigen;

typedef AD::Vector2ad<float> Vector2adf;
typedef AD::Vector3ad<float> Vector3adf;
typedef AD::Vector6ad<float> Vector6adf;
typedef AD::Matrix3ad<float> Matrix3adf;
typedef AD::DualValue_<float> DualValuef;
typedef AD::Isometry3ad<float> Isometry3adf;



template <typename Scalar_>
class ProjectPoint: public MultivariateFunction<Scalar_, 6, 3>{
public:
  void operator()(Scalar_* output, const Scalar_* input){

    // this maps the memory area in the input array to an
    // Eigen vector of dimension 6
    // if you feel uncomfy, you can
    // allocate an array
    //   Vector6<Scalar_> robot_pose;
    // fill the elements with a for loop
    //    for (int i=0; i<6; i++) robot_pose[i]=input[i];
    Eigen::Map<const Eigen::Matrix<Scalar_,6,1> > robot_pose(input);

    // this maps the memory area in the output array to an
    // Eigen vector of dimension 3
    // Changing the Eigen object would result
    // in doing side effect to the memory area
    Eigen::Map<Eigen::Matrix<Scalar_,3,1> > projected_point(output);

    
    // compute the rotation matrix and translation vector
    // encoded in robot_pose
    Isometry3<Scalar_> robot_pose_matrix=v2t<Scalar_>(robot_pose);
   
    // compute the positionof the point w.r.t. the world,
    // by multiplying it by a transformation matrix 
    projected_point=robot_pose_matrix*point;
  }

  // this is the parameter
  Eigen::Matrix<Scalar_, 3, 1> point;
};


// this is our function that supports the multivariate autodiff
ADMultivariateFunction<double, ProjectPoint> ad_project_point;

int main(int argc, char** argv){
  ad_project_point.point<<1,2,3;
  ad_project_point.point<<1,2,3;

  Eigen::Matrix<double, 6, 1> v;
  v << 0,0,0,0,0,0;
  
  Eigen::Matrix<double, 3, 1> output;
  Eigen::Matrix<double, 3, 6> jacobian;
  
  ad_project_point(&output[0], &v[0]);
  jacobian=ad_project_point.jacobian(&v[0]);

  cerr << "output: " << endl;
  cerr << output.transpose() << endl;
  cerr << "jacobian: " << endl;
  cerr << jacobian << endl;
}
