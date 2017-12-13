# generate a rotation matrix
A=rand(3,3);
[U,s,V]=svd(A);
R=V

# generate a translation vector
t=rand(3,1)

#generate 100 random points
P=100.*(rand(3,100)-.5);

#generate 3 noise matrices 3 levels of noise
N1=0.1*(rand(3,100)-.5);
N2=0.5*(rand(3,100)-.5);
N3=20.0*(rand(3,100)-.5);

#generate the set to match by applying R and t to X
#dimensions: (3,100)= (3,3)x(3x100) + (3,1)x(1x100)
X=R*P+t*ones(1,100);

sigmas = zeros(12,3);
R_est_rot= zeros(12,3);
#test icp with zero noise
disp('Zero Noise');
[R_est, t_est] = icp_relaxed(X,P);

[U_est, s_est, V_est]= svd(R_est);
sigmas(1:3,:)= s_est;
R_est_rot(1:3,:)= U_est*V_est';

disp('Low Noise');
#test icp with medium noise
[R_est, t_est] = icp_relaxed(X+N1,P);
[U_est, s_est, V_est]= svd(R_est);
sigmas(4:6,:)=s_est;
R_est_rot(4:6,:)= U_est*V_est';


disp('Medium Noise');
#test icp with medium noise
[R_est, t_est] = icp_relaxed(X+N2,P);
[U_est, s_est, V_est]= svd(R_est);
sigmas(7:9,:)=s_est;
R_est_rot(7:9,:)= U_est*V_est';


disp('Huge Noise');
#test icp with medium noise
[R_est, t_est] = icp_relaxed(X+N3,P);
[U_est, s_est, V_est]= svd(R_est);
sigmas(10:12,:)= s_est;
R_est_rot(10:12,:)= U_est*V_est';

sigmas
R
R_est_rot

                                %error between rot matrices
error= zeros(4,1);
for (i= 1:4)
  error(i,1)= trace( eye(3) - R'*R_est_rot((3*(i-1))+1:(3*(i-1))+3,:));
end

error

