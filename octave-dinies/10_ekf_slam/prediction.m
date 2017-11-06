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

  #domain spaces
  dimension_mu = size(mu, 1);
  dimension_u  = 2;

  #readability: the robot pose
  mu_r = mu(1:3);

  #get the control input u = [ux, uy, utheta]
  u = transition.v;

  #readability: current pose
  mu_x     = mu_r(1);
  mu_y     = mu_r(2);
  mu_theta = mu_r(3);

  #readability: current control input
  u_x     = u(1); #translational velocity
  u_theta = u(3); #rotational velocity

  c = cos(mu_theta);
  s = sin(mu_theta);


  #Jacobian A: States
  #initialize A as an identity and fill only the robot block
  A = eye(dimension_mu);
  A(1:3, 1:3)= [ 1, 0, -u_x*s;
                 0, 1,  u_x*c;
                 0, 0,  1    ];


  #Jacobian B: Controls
  #for each state variable we have to associate the available control inputs
  B = zeros(dimension_mu, dimension_u);
  B(1:3, 1:2)= [ c 0 ;
                 s 0 ;
                 0 1 ];

  #control noise u
  noise_v   = 0.1;     #constant part
  noise_u_1 = u_x;     #translational velocity dependent part
  noise_u_2 = u_theta; #rotational velocity dependent part

  #compose control noise covariance sigma_u
  sigma_u = [ noise_v^2+noise_u_1  0 ;
              0   noise_v^2+noise_u_2 ];

  #predict the robot motion, this is our f(x,u) function in the slides
  #the transition model only affects the robot pose not the landmarks
  mu(1:3) = transition_model(mu_r, u);

  #predict sigma
  sigma = A*sigma*A' + B*sigma_u*B';
end

