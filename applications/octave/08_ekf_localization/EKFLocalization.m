close all
clear
clc

addpath '../'
addpath '../tools/g2o_wrapper'
addpath '../tools/visualization'
source "../tools/utilities/geometry_helpers_2d.m"
addpath "./exercise" % uncomment this line to target the exercise
%addpath "./solution"

%load your own dataset dataset
[landmarks, poses, transitions, observations] = loadG2o('datasets/dataset_point.g2o');

%% init stuff
%initial pose
mu = rand(3,1)*20-10; 
mu(3) = normalizeAngle(mu(3));
printf('Random initial pose: [%f, %f, %f]\n', mu(1),mu(2), mu(3));
printf('\n***************** Press [SPACE] to perform a step *****************\n');
fflush(stdout);

%init covariance
sigma = eye(3)*0.001;

%init graphics
figure(1); title("ekf-localization");
plotState(landmarks, mu);


%simulation cycle
for i=1:length(transitions)

	%predict
	[mu, sigma] = prediction(mu, sigma, transitions(i));

	%correct
	[mu, sigma] = correction(mu, sigma, landmarks, observations(i));

	printf('current pose: [%f, %f, %f]\n', mu(1),mu(2), mu(3));
	fflush(stdout);	

	%plot current situation
	pause()
  %sigma multiplied factor [for visualization only!]
  f = 500;
	plotState(landmarks, mu, sigma*f, observations(i));
	hold off;
endfor

disp('mu = '), disp(mu');
