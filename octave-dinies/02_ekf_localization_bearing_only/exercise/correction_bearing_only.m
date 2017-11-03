#this function computes the update (also called correction)
#step of the filter
#inputs:
#  mu: mean, 
#  sigma: covariance of the robot (x,y.theta)
#  landmarks: a structure of landmarks, we can fetch the
#            position of a landmark given its index
#            by using the tools in the library
#  observations:
#            a structure containing n observations of landmarks
#            for each observation we have
#            - the index of the landmark seen
#            - the bearing angle where we have seen the landmark (theta) w.r.t the robot
#outputs:
#  [mu, sigma]: the updated mean and covariance

function [mu, sigma] = correction_bearing_only(mu, sigma, landmarks, observations)

  % determine how many landmarks we have seen
  num_landmarks_seen = length(observations.observation);
  
  % dimension of the state in dim, in our case is fixed to 3
  state_dim = size(mu,1);	

  %if I've seen no landmarks, i do nothing
  if (num_landmarks_seen==0)
    return;
  endif

  # we precompute some quantities that come in handy later on
  mu_x = mu(1);
  mu_y = mu(2);
  mu_theta = mu(3);
  c=cos(mu_theta);
  s=sin(mu_theta);
  R = [c -s; s c]; # rotation matrix
  Rt=[c,s;-s c]; # transposed rotation matrix
  Rtp=[-s,c;-c,-s]; # derivative of transposed rotation matrix
		
  # here in one go, we go through all landmark measurements vector
  # for each landmark, we assemble
  # the "total" measurement vector, containing all stacked measures
  # the "total" prediction vector, containing all staked predictions
  # the "total" jacobian, consisting of all jacobians of predictions stacked
  # octave allows to "Add" rows to a matrix, thus we dynamically resize them

  for i=1:num_landmarks_seen
    %retrieve info about the observed landmark
    measurement = observations.observation(i);

    z_t(end+1,:) = measurement.bearing; % where we see the landmark

    current_land = searchById(landmarks, measurement.id);
    lx = current_land.x_pose; % its absolute (true) position
    ly = current_land.y_pose;

    %where I should see that landmark
    t= [lx - mu_x ; ly - mu_y];
    bearing_measurement= Rt*t;
    bearing_prediction = atan2(bearing_measurement(2), bearing_measurement(1));

    h_t(end+1,:) = bearing_prediction;

    %compute its Jacobian
    denom= bearing_measurement(1)^2 + bearing_measurement(2)^2;
    d_atan= [ -bearing_measurement(2)/denom  bearing_measurement(1)/denom];
    d_g= [ -Rt Rtp*t ];
    C= d_atan*d_g;

    C_t(end+1,:) = C;

  endfor

  %observation noise
  noise = 0.01;
  sigma_z =eye(num_landmarks_seen)*noise^2 ;

  %Kalman gain
  K = sigma*C_t'*inverse(sigma_z + C_t*sigma*C_t');

  %update mu
  mu = mu +K*(z_t-h_t);

  %update sigma
  sigma =(eye(3)-K*C_t)*sigma;

end
