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

function [mu, sigma] = prediction(mu, sigma, control_input)

  #domain spaces
  dimension_mu = size(mu, 1);
  dimension_u  = 2;

  #get the control input u = [ux, uy, utheta]
  u = control_input.v;

  #predict the robot motion, this is our f(x,u) function in the slides
  #the transition model only affects the robot pose not the landmarks
  mu_r = transition_model(mu(1:3), u);

  #update the robot state
  mu(1:3) = mu_r;

  #readability: current pose
  mu_x     = mu_r(1);
  mu_y     = mu_r(2);
  mu_theta = mu_r(3);

  #readability: current control input
  u_x     = u(1); #translational velocity
  u_theta = u(3); #rotational velocity

  #Jacobian A: df(x, u)/dx
  #initialize A as an identity and fill only the robot block
  A = eye( #TODO: set full state/state dimensions
  A(1:3,1:3) = #TODO: set portion that is affected by transition function

  #Jacobian B: df(x, u)/du
  #for each state variable we have to associate the available control inputs
  B = zeros( #TODO: set full control/state dimensions
  B(1:3,:) = #TODO: set portion that is affected by transition function

  #control noise u: standard deviations
  sigma_u = 0.1;     #constant part
  sigma_T = u_x;     #translational velocity dependent part
  sigma_R = u_theta; #rotational velocity dependent part

  #compose control noise covariance sigma_u
  sigma_u = #TODO: set noise covariance

  #predict sigma
  sigma = #TODO: set full state sigma (consisting of state and control variables)
endfunction

