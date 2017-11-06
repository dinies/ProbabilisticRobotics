close all
clear
clc

#load dependencies
addpath "../../"
addpath "../tools/g2o_wrapper"
addpath "../tools/visualization"
source "../tools/utilities/geometry_helpers_2d.m"

#load your own dataset dataset, without landmarks (first entry remains empty)
[_, poses, transitions, observations] = loadG2o("../datasets/dataset_point.g2o");

#set initial pose at the origin - we don't know the map and neither our location
mu = [0;  #x coordinate
      0;  #y coordinate
      0]; #orientation theta (yaw angle)
 #printf("initial pose: [%f, %f, %f]\n", mu(1), mu(2), mu(3));

#initialize covariance: high value means high uncertainty
sigma = eye(3);

#data association: to and from mapping between EKF state and landmark identifiers
#all mappings are initialized with invalid value -1 (meaning that the index is not mapped yet)
#since we do not know how many landmarks we will observe, we allocate a large enough buffer
state_to_id_map = ones(10000, 1)*-1; #this is j(m)

#in order to retrieve a landmarks position in our state as we observe it,
#we need to establish a mapping from landmark identifiers to the state
id_to_state_map = ones(10000, 1)*-1; #this is j^-1(m)

#initialize GUI with initial situation
figure("name", "ekf_slam",    #figure title
       "numbertitle", "off"); #remove figure number
trajectory = [mu(1), mu(2)];
plotStateEKFSLAM(mu, sigma, [], state_to_id_map, trajectory);

#simulation cycle: for the number of transitions recorded in the dataset
for t = 1:length(transitions)

  #obtain current transition
  transition = transitions(t);
  
  #obtain current observation
  observation = observations(t);

  #EKF predict TODO
  [mu, sigma] = prediction(mu, sigma, transition);

  #EKF correct TODO
  [mu, sigma, id_to_state_map, state_to_id_map] = correction(mu, sigma, observation, id_to_state_map, state_to_id_map);

  #display current state and wait briefly
  #printf("current pose: [%f, %f, %f]\n", mu(1), mu(2), mu(3));
  trajectory = [trajectory; mu(1), mu(2)];
  plotStateEKFSLAM(mu, sigma, observation, state_to_id_map, trajectory);
  pause(.1)
  fflush(stdout);	
endfor

