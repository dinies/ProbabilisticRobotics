#include "localizer.h"
#include <cmath>
#include <iostream>
namespace thin_localizer {

  using namespace std;


  Particle::Particle(){
    _pose.setZero();
    _weight = 1.0f;
  }


  Localizer::Localizer(){
    _min_update_translation=0.1;
    _min_update_rotation=0.1;

    // transition_model
    _noise_coeffs << 
      0.01, 0.0005, 0.0002,
      0.0005, 0.0001, 0.0001,
      0.001, 0.00001, 0.01;

    // observation model
    _min_valid_points = 30;
    _min_weight=0.1;
    _particle_resetting = true;
    _likelihood_gain = 10;
  }


  void Localizer::setMap(const UnsignedCharImage& m, 
			 float resolution,
			 unsigned char occ_threshold,
			 unsigned char free_threshold) {
    _resolution=resolution;
    _inverse_resolution = 1./resolution;
    _map=m.clone();

    // creates an integer matrix where the cells are either
    // -1 (occupied)
    // -2 (unknown)
    // k>0 an unique integer that represent an obstacle in the map.
    _int_map.create(m.rows, m.cols);
    int k = 0;
    int free_count = 0, occ_count = 0, unknown_count = 0;
  
    for (int r = 0; r<m.rows; r++){
      const unsigned char* src_ptr = m.ptr<const unsigned char>(r);
      int* dest_ptr = _int_map.ptr<int>(r);
      for (int c = 0; c<m.cols; c++) {
	unsigned char cell=*src_ptr;
	int v=-1;
	if (cell<occ_threshold) {
	  occ_count++;
	  v=k++;
	} else if (cell>free_threshold) {
	  free_count++;
	  v=-1;
	} else {
	  unknown_count++;
	  v=-2;
	}
	*dest_ptr=v;
	dest_ptr++;
	src_ptr++;
      }
    }
    cerr << "free: " << free_count << endl;
    cerr << "unknown: " << unknown_count << endl;
    cerr << "occupied: " << occ_count << endl;
    init();
  }



  void Localizer::init(int num_particles,
		       float dmax, 
		       float robot_radius,
		       float min_weight,
		       int min_valid_points){
    _particles.resize(num_particles);
    _robot_radius = robot_radius;
    _min_valid_points = min_valid_points;

    // computes the distance map
    // _int_map is the integer map set through setMap
    // _distances are floats representing the distances (in meters)
    // between a cell and the closest obstacles
    // The distances are computed also for the unknown cells
    // but stored as  their opposite.
    // This allows doscriminating between free and unknown cells
    // when computing the likelihood
    makeDistanceMap(_distance_map, _assoc_map, _distances,
		    _int_map, dmax*_inverse_resolution);
    _distances*=_resolution;
    _free_cells.clear();
    for (int r = 0; r<_map.rows; r++){
      for (int c = 0; c<_map.cols; c++) {
	if (_distances.at<float>(r,c)>_robot_radius)
	  _free_cells.push_back(Eigen::Vector2f(_resolution*r, _resolution*c));
      }
    }

    dmap2img(_distance_map_image, _distance_map);
  }


  Eigen::Vector3f Localizer::sampleFromFreeSpace() {
    int r=drand48()*(_free_cells.size()-1);
    float theta = (drand48()-0.5)*2*M_PI;
    return Eigen::Vector3f(_free_cells[r].x(), _free_cells[r].y(), theta);
  }

  void Localizer::startGlobal() {
    for (size_t i = 0; i<_particles.size(); i++){
      _particles[i]._pose = sampleFromFreeSpace();
      _particles[i]._weight = 1;
    }
  }

  void Localizer::setPose(const Eigen::Vector3f pose, Eigen::Vector3f standard_deviations) {
    
    Eigen::Isometry2f pose_transform = v2t(pose);
    for (size_t i = 0; i<_particles.size(); i++){
      Eigen::Vector3f noise;
      for (int k = 0; k<3; k++)
	noise[k] = standard_deviations[k]*_normal_generator(_random_generator);
      _particles[i]._pose = t2v(pose_transform*v2t(noise));
      _particles[i]._weight = 1;
    }
  }

  void Localizer::predict(const Eigen::Vector3f control) {
    if (control.squaredNorm()==0)
      return;
    prepareSampling(control);
    for (size_t i = 0; i<_particles.size(); i++)
      _particles[i]._pose = sample(_particles[i]._pose);
    _cumulative_translation+=control.head<2>().norm();
    _cumulative_rotation+=fabs(control[2]);
  }


  //! uniform resampling algorithm
  //! indices: the indices of the particles that survive after resampling
  //! weights: the weights of the particles as input
  //! n: the number of particles
  void resample_uniform(int* indices, const double* weights, int n){
    double acc=0;
    const double* w = weights;
    for (int i=0; i<n; i++, w++) {
      acc+= *w;
    }
    double inverse_acc = 1./acc;
    double cumulative_value=0;
    double step = 1./n;
    double threshold = step * drand48();
    int* idx = indices;
    
    w=weights;
    int k=0;
    for (int i=0; i<n; i++, w++){
      cumulative_value += (*w) *inverse_acc;
      while(cumulative_value>threshold){
	*idx = i;
	idx++;
	k++;
	threshold += step;
      }
    }
  }

  bool Localizer::update(const Vector2fVector observation){
    // if the platform did not move enough, do nothing
    if (_cumulative_rotation<_min_update_rotation &&
	_cumulative_translation < _min_update_translation)
      return 0;

    _last_endpoints = observation; 
    //update
    _cumulative_translation = 0;
    _cumulative_rotation = 0;
    _cumulative_likelihood = 0;
    for (size_t i = 0; i<_particles.size(); i++) {
      // compute the weight of each particle
      float w = likelihood(_particles[i]._pose, observation);
      // if the weight is 0 and replace the particle with a random one,
      // otherwise assign a weight to a particle, based on the likelihood
      if (w==0 && _particle_resetting) {
	w=_min_weight;
	_particles[i]._pose = sampleFromFreeSpace();
      } 
      _particles[i]._weight = w;
      _cumulative_likelihood += w;;
    }

    if (_cumulative_likelihood < 0)
      return false;

    // resample
    int indices[_particles.size()];
    double weights[_particles.size()];
    for (size_t i = 0 ; i<_particles.size(); i++)
      weights[i]=_particles[i]._weight;
    resample_uniform(indices, weights, _particles.size());
    if (_cumulative_likelihood==0) {
       return true;
    }
    ParticleVector aux(_particles.size());
    int* idx = indices;
    for (size_t i=0; i<_particles.size(); i++){
      aux[i]=_particles[*idx];
      aux[i]._weight=1;
      idx++;
    }
    _particles=aux;
    return true;
  }

  void Localizer::paintState(RGBImage& img, bool use_distance_map){
    if (use_distance_map)
      cvtColor(_distance_map_image, img, CV_GRAY2BGR);
    else
      cvtColor(_map, img, CV_GRAY2BGR);
   float ires=1./_resolution;
    int count=0;
    for (size_t i=0; i<_particles.size(); i++){
      int r = _particles[i]._pose.x()*ires;
      int c = _particles[i]._pose.y()*ires;
      if (r<0||r>=img.rows||c<0||c>=img.cols)
	continue;
      count++;
      img.at<cv::Vec3b>(r,c)=cv::Vec3b(0,0,255);
    }

    computeStats();
    Eigen::Isometry2f mean_transform=v2t(_mean);

    for (size_t i=0; i<_last_endpoints.size(); i++){
      Eigen::Vector2f ep=mean_transform*_last_endpoints[i];
      int r = ep.x()*ires;
      int c = ep.y()*ires;
      if (r<0||r>=img.rows||c<0||c>=img.cols)
	continue;
      img.at<cv::Vec3b>(r,c)=cv::Vec3b(255,0,0);
    }
  }

  void Localizer::prepareSampling(const Eigen::Vector3f& control) {
    Eigen::Vector3f scales(fabs(control[0]),fabs(control[1]),fabs(control[2])); 
    _std_deviations=_noise_coeffs*scales;
    for (size_t i = 0; i<3; i++){
      _std_deviations[i]=sqrt(_std_deviations[i]);
    }
    _last_control=control;
  }

  Eigen::Vector3f Localizer::sample(const Eigen::Vector3f& old_state) {
    Eigen::Vector3f noise;
    for (int i = 0; i<3; i++)
      noise[i] = _std_deviations[i]*_normal_generator(_random_generator);
    
    noise += _last_control;
    Eigen::Isometry2f transform=v2t(old_state) * v2t(noise);
    return t2v(transform);
  }


  void Localizer::computeStats(){
    Eigen::Vector2f translational_mean;;
    Eigen::Vector2f angular_mean;
    translational_mean.setZero();
    angular_mean.setZero();
    // computes the mean. To calculate the angular component sum
    // the vectors (cos(theta), sin(theta) and recover the global
    // orientation
    for (size_t i = 0; i<_particles.size(); i++) {
      translational_mean+=_particles[i]._pose.head<2>();
      float theta=_particles[i]._pose[2];
      angular_mean+=Eigen::Vector2f(cos(theta), sin(theta));
    }
    _mean.head<2>()=translational_mean * (1./_particles.size());
    _mean[2]=atan2(angular_mean.y(),angular_mean.x());

    _covariance.setZero();
    for (size_t i = 0; i<_particles.size(); i++) {
      Eigen::Vector3f dp = _particles[i]._pose-_mean;
      dp[2]=fmod(dp[2], 2*M_PI);
      if (dp[2]>M_PI)
	dp[2]-=M_PI;
      _covariance+=dp*dp.transpose();
    }
    _covariance*=1./_particles.size();
  }

  double Localizer::likelihood(const Eigen::Vector3f& pose, const Vector2fVector& observation){
    Eigen::Isometry2f iso=v2t(pose);
  
    // handle the robot out of the map
    Eigen::Vector2i p = world2grid(iso.translation());
    if(p.x()<0 || p.x()>=_distances.rows || p.y() <0 || p.y()>= _distances.cols)
      return 0;

    // handle the robot in the unknown
    float d=_distances.at<float>(p.x(),p.y());
    if (d<_robot_radius)
      return 0;
  
    // handle the beams
    float cumulative_distance = 0;
    int valid_points=0;
    for (size_t i =0; i<observation.size(); i++) {
      p = world2grid(iso*observation[i]);
      if(p.x()<0 || p.x()>=_distances.rows || p.y() <0 || p.y()>= _distances.cols)
	continue;
      cumulative_distance+=fabs(_distances.at<float>(p.x(),p.y()));
      valid_points++;
    }
    // if too less beams are good, ignore
    if (valid_points< _min_valid_points)
      return _min_weight;

    // heuristic but effective likelihood
    cumulative_distance/=valid_points;
    cumulative_distance*=_likelihood_gain;
    return exp(-cumulative_distance)+_min_weight;
  }

}
