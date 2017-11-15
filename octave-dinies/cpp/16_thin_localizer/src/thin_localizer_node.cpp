#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sys/time.h>
#include <sstream>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "localizer.h"

using namespace std;
using namespace thin_localizer;

tf::TransformListener* listener=0;
Localizer localizer;
Eigen::Vector3f old_pose;
bool restarted = true;
std::string laser_topic = "/base_scan";
double cumulative_time = 0;
int counter = 0;
float forced_max_range = 10;
float squared_endpoint_distance = 0.1*0.1;
bool show_distance_map=false;

//! returns the time in milliseconds
double getTime() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return 1e3*tv.tv_sec+1e-3*tv.tv_usec;
}

Eigen::Vector3f convertPose(tf::StampedTransform t) {
  double yaw,pitch,roll;
  tf::Matrix3x3 mat =  t.getBasis();
  mat.getRPY(roll, pitch, yaw);
  return Eigen::Vector3f(t.getOrigin().x(), t.getOrigin().y(), yaw);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  //std::string frame_id = msg->header.frame_id;
  ros::Time t = msg->header.stamp;
  std::string error;
  if (! listener->waitForTransform ("/odom", "/base_link", t, ros::Duration(0.5), ros::Duration(0.5), &error)) {
    cerr << "error: " << error << endl;
    return;
  }
  tf::StampedTransform robot_pose_t;
  listener->lookupTransform("/odom", "/base_link", t, robot_pose_t);
  Eigen::Vector3f robot_pose = convertPose(robot_pose_t);
   
  if (! listener->waitForTransform ("/base_link", msg->header.frame_id, t, ros::Duration(0.5), ros::Duration(0.01), &error)) {
    cerr << "error: " << error << endl;
    return;
  }
  double t0=getTime();
  tf::StampedTransform laser_pose_t;
  listener->lookupTransform("/base_link", msg->header.frame_id, t, laser_pose_t);
  Eigen::Vector3f laser_pose = convertPose(laser_pose_t);

  if (! restarted) {
    Eigen::Vector3f control=t2v(v2t(old_pose).inverse()*v2t(robot_pose));
    localizer.predict(control);
    counter = 0;
    cumulative_time=0;
  }

  old_pose=robot_pose;

  // we have the transforms, we can start assembling the endpoints for the localizer
  // in doing that we do take care that no endpoint is closer than
  // squared_endpoint_distance from its predecessor
  // this avoids unnecessary computation when in crowded settings
  Eigen::Isometry2f laser_transform=v2t(laser_pose);
  Vector2fVector endpoints(msg->ranges.size());
  int k = 0;
  double angle=msg->angle_min-msg->angle_increment;
  float max_range = (msg->range_max<forced_max_range) ? msg->range_max : forced_max_range;
  Eigen::Vector2f last_endpoint(-1000, -1000);
  for (size_t i=0; i<msg->ranges.size(); i++){
    float r=msg->ranges[i];
    angle+=msg->angle_increment;
    if (r<msg->range_min)
      continue;
    if (r>=max_range)
      continue;
    Eigen::Vector2f dir(cos(angle), sin(angle));
    Eigen::Vector2f ep=laser_transform*(dir*r);
    Eigen::Vector2f delta = last_endpoint-ep;
    if (delta.squaredNorm()>squared_endpoint_distance) {
      endpoints[k]=ep;
      last_endpoint = ep;
      k++;
    }
  }
  endpoints.resize(k);
  bool updated = localizer.update(endpoints);
  if (updated || restarted) {
    RGBImage img;
    localizer.paintState(img, show_distance_map);
    cv::imshow("localizer", img);
  
  }  
  double t1=getTime();

  cumulative_time+=t1-t0;
  counter++;

  restarted=false;
  // if (! (counter&10))
  //   cerr << endl << "computation took: " << cumulative_time/counter << " ms per round" << endl;
  if (updated)
    cerr << "cl: " << localizer.cumulativeLikelihood() << endl;
  char key=cv::waitKey(10);
  switch(key) {
  case 'g': 
    cerr << "starting global localization" << endl;
    restarted = true;
    localizer.startGlobal();
    break;
  case 'm': 
    cerr << "toggling map type" << endl;
    show_distance_map=!show_distance_map;
    break;
  case 'r': 
    localizer.setParticleResetting(! localizer.particleResetting());
    cerr << "particle resetting = " << localizer.particleResetting();
    break;
  default:;
  }

}


int main(int argc, char **argv){
  UnsignedCharImage img = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  localizer.setMap(img, 0.05, 10, 230);
  localizer.init(5000, 3, 0.2, 300);
  localizer.startGlobal();
  ros::init(argc, argv, "thin_localizer_node");
  ros::NodeHandle n;
  listener = new tf::TransformListener;
  ros::Subscriber sub = n.subscribe(laser_topic, 100, laserCallback);
  ros::spin();

  return 0;
}
