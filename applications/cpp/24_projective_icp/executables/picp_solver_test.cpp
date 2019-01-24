#include <assert.h>
#include "camera.h"
#include "utils.h"
#include "points_utils.h"
#include "picp_solver.h"

using namespace std;
using namespace pr;


const char* banner[]={
  "p3p_solver_finder_test",
  " demonstrates a simple projective icp least squares solver",
  " move the observer with [W,A,S,D]",
  " spacebar to trigger one optimization round",
  " ESC to quit",
  0
};

void computeFakeCorrespondences(IntPairVector& correspondences,
				const Vector2fVector reference_image_points,
				const Vector2fVector current_image_points){
  correspondences.resize(current_image_points.size());
  int num_correspondences=0;
  assert(reference_image_points.size()==current_image_points.size());
  
  for (size_t i=0; i<reference_image_points.size(); i++){
    const Eigen::Vector2f& reference_point=reference_image_points[i];
    const Eigen::Vector2f& current_point=current_image_points[i];
    IntPair& correspondence=correspondences[num_correspondences];
    if (reference_point.x()<0 || current_point.x()<0)
      continue;
    correspondence.first=i;
    correspondence.second=i;
    num_correspondences++;
  }
}

int main (int argc, char** argv) {
  printBanner(banner);

  Vector3fVector world_points;
  Eigen::Vector3f lower_left_bottom(-10,-10,-10);
  Eigen::Vector3f upper_right_top(10,10,10);
  int num_segments=20;
  float density=10; // 10 points per meter
  makeWorld(world_points,
	    lower_left_bottom, 
	    upper_right_top,
	    num_segments,
	    density);

  cerr << "Generated Model with " << world_points.size() << " points" << endl;

  // we test the projection
    
  Eigen::Matrix3f camera_matrix;
  camera_matrix << 
    150, 0, 320,
    0, 150,240,
    0, 0, 1;
  
  int rows=480;
  int cols=640;
  Camera cam(rows, cols, camera_matrix);
  Vector2fVector reference_image_points;

  // the bottom line projects all points of the image in the current frame
  cam.projectPoints(reference_image_points, world_points, true); 
  
  char key=0;

  // construct a solver
  PICPSolver solver;
  solver.setKernelThreshold(10000);

  while (key!=OPENCV_KEY_ESCAPE) {
    // project the points on the image plane
    RGBImage shown_image(rows,cols);
    shown_image=cv::Vec3b(255,255,255);

    Vector2fVector current_image_points;
    const bool keep_indices=true;
    double t_start_projection=getTime();
    cam.projectPoints(current_image_points, world_points, keep_indices);
    double t_end_projection=getTime();
    
    IntPairVector correspondences;
    computeFakeCorrespondences(correspondences, reference_image_points, current_image_points);
    
    drawPoints(shown_image,reference_image_points,cv::Scalar(0,0,255),3);
    drawPoints(shown_image,current_image_points,cv::Scalar(255,0,0),3);
    drawCorrespondences(shown_image,
		       reference_image_points,
		       current_image_points,
		       correspondences,
		       cv::Scalar(0,255,0));

    cv::imshow("picp_solver_test", shown_image);
    Eigen::Isometry3f motion=Eigen::Isometry3f::Identity();
    float dr=0.05;
    float dt=0.1;
    key=cv::waitKey(0);
    double t_start_solve=0;
    double t_end_solve=0;
    switch(key) {
    case 'w': motion.translation()=Eigen::Vector3f(0,0,-dt); break;
    case 's': motion.translation()=Eigen::Vector3f(0,0,dt); break;
    case 'a': motion.linear()=Ry(dr); break;
    case 'd': motion.linear()=Ry(-dr); break;
    case ' ': {
      motion.setIdentity();
      t_start_solve=getTime();
      solver.init(cam,world_points,reference_image_points);
      solver.oneRound(correspondences,false);
      cam=solver.camera();
      t_end_solve=getTime();
      break;
    }  
    default: break;
    }
    cam.setWorldToCameraPose(motion*cam.worldToCameraPose());
    cerr << "projection took: " << (t_end_projection-t_start_projection) << " ms" << endl;
    cerr << "solve took: " << (t_end_solve-t_start_solve) << " ms" << endl;
  }
  return 0;
}
