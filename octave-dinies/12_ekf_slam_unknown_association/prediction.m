#this function implements the kalman prediction step of our SLAM system
# inputs: 
#   transition: is a data structure containing several things,
#               coming from the simulator
#               of these we are interested only in the offset
#               accessible through the "v" field of the structure
#               
#               transition.v(1)=offset x
#               transition.v(2)=offset y (ignore for a diff drive robot)
#               transition.v(3)=offset theta 
#
#  considering as 'm' the number of seen landmarks
#  mu: is the mean of (x,y,theta, l1, l2, ..., lm), i.e. the previously estimated robot pose
#      and the m landmark positions
#  sigma: is the mean of the previously estimated robot pose and landmark positions ((3+m)x(3+m) matrix)

# outputs 
# [mu, sigma] are mean and covariance of the estimate after transition

function [mu, sigma] = prediction(mu, sigma, transition)

	state_dim = size(mu,1);
	input_dim = 2;
	mu_robot = mu(1:3);
	

	u = transition.v;
	%it returns u = [ux, uy, utheta]. simply not consider uy

	%predict mu // this is our f(x,u) function in the slides
	% the transition model only affects the robot pose
	mu_robot = transition_model(mu_robot, u);
	mu(1:3) = mu_robot;

	mu_x = mu_robot(1);
	mu_y = mu_robot(2);
	mu_theta = mu_robot(3);

	u_x = u(1);
	u_theta = u(3);
	
	s = sin(mu_theta);
	c = cos(mu_theta);

	%Jacobian A
	%initialize A as an identity and fill only the robot block
	A = eye(state_dim,state_dim);
	A(1:3,1:3) = [	1, 0, -u_x*s;
		        0, 1, u_x*c;
			0, 0, 1];

	%Jacobian B
	B = zeros(state_dim, input_dim);
	B(1:3,:) = [ c, 0;
		     s, 0;
		     0, 1];

	
	%motion noise
	noise = 0.1; 			%constant part	
	v_noise = u_x^2; 		%lin vel dependent part
	w_noise = u_theta^2;		%ang vel dependent part

	sigma_u = [ noise+v_noise, 0;
		0, noise+v_noise];


	%predict sigma
	sigma = A*sigma*A' + B*sigma_u*B';

end
