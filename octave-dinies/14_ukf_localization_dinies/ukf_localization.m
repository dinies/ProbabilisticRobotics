close all
clear
clc

#load dependencies
addpath "../../"
addpath "../tools/g2o_wrapper"
addpath "../tools/visualization"
source "../tools/utilities/geometry_helpers_2d.m"

%load dataset
[landmarks, poses, transitions, observations] = loadG2o('../datasets/dataset_point.g2o');

%% init stuff
%initial pose
mu =  rand(3,1)*10-5; 
mu(3) = normalizeAngle(mu(3));
printf('Random initial pose: [%f, %f, %f]\n', mu(1),mu(2), mu(3));
fflush(stdout);

%init covariance
sigma = eye(3)*0.001;

%init graphics
figure(1); title("ukf-localization");
plot_state(landmarks, mu);


%simulation cycle
for i=1:length(transitions)
	%predict
	[sigmaP, wM, wC] = prediction(mu, sigma, transitions(i));

	%correct
	[mu, sigma] = correction(sigmaP, wM, wC, landmarks, observations(i));

	printf('current pose: [%f, %f, %f]\n', mu(1),mu(2), mu(3));
	fflush(stdout);

	%plot current situation
	pause(.1)
	plot_state(landmarks, mu, sigma, observations(i));
	hold off;

endfor

disp('mu = '), disp(mu');

