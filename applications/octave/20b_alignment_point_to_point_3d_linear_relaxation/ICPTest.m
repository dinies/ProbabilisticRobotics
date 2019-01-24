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
N3=2.0*(rand(3,100)-.5);

#generate the set to match by applying R and t to X
X=R*P+t*ones(1,100);

#test icp with zero noise
disp('Zero Noise');
[R_est, t_est] = icp(X,P)

disp('Low Noise');
#test icp with medium noise
[R_est, t_est] = icp(X+N1,P)

disp('Medium Noise');
#test icp with medium noise
[R_est, t_est] = icp(X+N2,P)

disp('Huge Noise');
#test icp with medium noise
[R_est, t_est] = icp(X+N3,P)

