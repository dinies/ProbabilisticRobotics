#include "camera.h"
#include "utils.h"
#include "points_utils.h"

using namespace std;
using namespace pr;

const char* banner[]={
  "camera_test",
  " demonstrates a simple pinhole camera model on a virtual world",
  " move the observer with [W,A,S,D]",
  " ESC to quit",
  0
};


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
  
  char key=0;
  const char ESC_key=27;
  while (key!=ESC_key) {
    Vector2fVector image_points;
    // project the points on the image plane
    double t_start_projection=getTime();
    cam.projectPoints(image_points, world_points, false);
    double t_end_projection=getTime();
    RGBImage shown_image(rows,cols);
    shown_image=cv::Vec3b(255,255,255);
    drawPoints(shown_image,image_points,cv::Scalar(255,0,0),3);
    cv::imshow("camera_test", shown_image);
    Eigen::Isometry3f current_pose=cam.worldToCameraPose();
    Eigen::Isometry3f motion=Eigen::Isometry3f::Identity();
    float dr=0.05;
    float dt=0.1;
    key=cv::waitKey(0);
    switch(key) {
    case 'w': motion.translation()=Eigen::Vector3f(0,0,-dt); break;
    case 's': motion.translation()=Eigen::Vector3f(0,0,dt); break;
    case 'a': motion.linear()=Ry(dr); break;
    case 'd': motion.linear()=Ry(-dr); break;
    default: ;
    }
    cam.setWorldToCameraPose(motion*current_pose);
    cerr << "projection took: " << (t_end_projection-t_start_projection) << " ms" << endl;
  }
  return 0;
}
