#generate a bunch of sample points

source "./icp_3d.m"

n_points=100;
P_world=(rand(3,n_points)-0.5)*10;

#ideal position of world w.r.t robot
x_true=[0, 0, 0, pi/2, pi/6, pi]';
X_true=v2t(x_true);

#compute the measurements by mapping them in the observer frame 
P_world_hom=ones(4, n_points);
P_world_hom(1:3, :)=P_world;
Z_hom=X_true*P_world_hom;
Z=Z_hom(1:3,:);

noise_sigma=1
# add some noise
# Z(1:3,:)+=normrnd(0,noise_sigma, 3, n_points);


iterations=100;
damping=0; # damping factor

chi_stats=zeros(2,iterations);
inliers_stats=zeros(2,iterations);

# test with a good initial guess
x_guess=x_true+[0.5,0.5,0.5,0.1,0.1,0.1]';

[x_result, chi_stats(1,:),  inliers_stats(1,:)] = doICP(x_guess, P_world, Z, iterations, damping, 1e9);
X_guess=v2t(x_guess);

t=figure(1);
title('good guess plain implementation');
xlabel('iteration');
ylabel('error (less is better)');
hold on
plot(log(chi_stats(1,:)+1),"-b",'LineWidth',3)
hold off

# test with a bad initial guess
x_guess=[0,0,0,0,0,0]';

[x_result, chi_stats(2,:),  inliers_stats(2,:)] = doICP(x_guess, P_world, Z, iterations, damping, 1e9);
X_guess=v2t(x_guess);

t=figure(2);
title('good and bad guess, plain implementation');
xlabel('iteration');
ylabel('error (less is better)');
hold on
plot(log(chi_stats(1,:)+1),"-b",'LineWidth',3)
plot(log(chi_stats(2,:)+1),"-b",'LineWidth',3)
hold off

#test with a linear relaxation of ICP
x_guess=[0,0,0,0,0,0]';

[x_result, chi_stats(2,:),inliers_stats(2,:)]=icp_linrel(x_guess,P_world,Z,iterations,damping,1e9);


pause(10);
