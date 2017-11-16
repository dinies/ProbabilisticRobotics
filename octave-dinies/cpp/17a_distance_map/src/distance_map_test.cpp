#include "distance_map.h"
using namespace thin_navigation;
using namespace std;

void convertDistanceMap(RGBImage& img, FloatImage& distance_image, float threshold) {
    img.create(distance_image.rows, distance_image.cols);
    img=cv::Vec3b(255,0,0);
    
    float mdist = 0;
    for (int r=0; r<distance_image.rows; r++)
        for (int c=0; c<distance_image.cols; c++){
          float dist=distance_image.at<float>(r,c);  
	  mdist = (mdist < dist) ?  dist : mdist;
        }
    mdist = std::sqrt(mdist);
    cerr << "mdist:" << mdist << endl;
            
    // cerr << "mdist=" << mdist;
    for (int r=0; r<distance_image.rows; r++)
        for (int c=0; c<distance_image.cols; c++){
            float dist=distance_image.at<float>(r,c);
	    if (dist>threshold)
	      continue;
	    float ndist = 127 * std::sqrt(dist)/mdist;
            int v = 127-ndist;
            img.at<cv::Vec3b>(r,c) = cv::Vec3b(v,v,v);
        }
}

int main(int argc, char** argv){
  int rows=480;
  int cols=640;
  int num_points=100;

  // generate n random points
  Vector2iVector points(num_points);
  for (size_t i=0; i<points.size(); i++){
    points[i]=Eigen::Vector2i(rows*drand48(), cols*drand48());
  }
  
  // create an image containing the random points;
  IntImage points_image(rows, cols);
  points_image=-1;

  // copy the points in the image
  for (size_t i=0; i<points.size(); i++){
    const Eigen::Vector2i& point=points[i];
    int r=point.x();
    int c=point.y();
    points_image.at<int>(r,c)=i;
  }

  DistanceMap distance_map;
  FloatImage distance_image(rows, cols);
  IntImage indices_image(rows,cols);
  RGBImage shown_image;

  float d_max=100;
  cv::namedWindow("distance map");
  
  int d_curr=0;

  // show progressive construction of distance map
  while (d_curr<d_max) {
    distance_map.compute(indices_image, distance_image, points_image, d_curr);
    convertDistanceMap(shown_image, distance_image, d_curr-1);
    cv::imshow("distance map", shown_image);
    cv::waitKey(10);
    d_curr++;
  }



}
