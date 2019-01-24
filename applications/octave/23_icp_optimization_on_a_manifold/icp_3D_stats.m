#generate a bunch of sample points

source "./icp_3D_nomanifold.m"
source "./icp_3D_manifold.m"

n_points=100;
P_world=(rand(3,n_points)-0.5)*100;

#ideal position of world w.r.t robot
x_true=[30, 10, -20, pi/6, pi/6, pi/6]';
X_true=v2t(x_true);

#compute the measurements by mapping them in the observer frame 
P_world_hom=ones(4, n_points);
P_world_hom(1:3, :)=P_world;
Z_hom=X_true*P_world_hom;
Z=Z_hom(1:3,:);

noise_sigma=10
# add some noise
Z(1:3,:)+=normrnd(0,noise_sigma, 3, n_points);


iterations=100;
damping=0; # damping factor

outlier_rounds=4;
chi_manifold_stats=zeros(outlier_rounds,iterations);
chi_manifold_rk_stats=zeros(outlier_rounds,iterations);

inliers_manifold_stats=zeros(outlier_rounds,iterations);
inliers_manifold_rk_stats=zeros(outlier_rounds,iterations);


# test with a bad initial guess
x_guess=[0,0,0,0,0,0]';

for (i=0:outlier_rounds)
  printf("round %d\n",i);
  #corrupt the measurements with a fraction of outliers
  wrong_points=(n_points/outlier_rounds)*i;
  if (wrong_points>0)
    Z(:,1:wrong_points)=(rand(3,wrong_points)-0.5)*100;
  endif

  #without robust kernel
  X_guess=v2t(x_guess);
  [X_result, chi_manifold_stats(i+1,:),  inliers_manifold_stats(i+1,:)] = doICPManifold(X_guess, P_world, Z, iterations,damping, 1e9);

  #with robust kernel
  X_guess=v2t(x_guess);
  [X_result, chi_manifold_rk_stats(i+1,:),  inliers_manifold_rk_stats(i+1,:)] = doICPManifold(X_guess, P_world, Z, iterations, damping, 20);
endfor;

t=figure(1);
title('error without robust kernels');
xlabel('iteration');
ylabel('error (less is better)');
hold on 
plot(log(chi_manifold_stats(1,:)+1),"-r",'LineWidth',3)
plot(log(chi_manifold_stats(2,:)+1),"-r",'LineWidth',3)
plot(log(chi_manifold_stats(3,:)+1),"-r",'LineWidth',3)
hold off

t=figure(4);
title('error with robust kernels');
xlabel('iteration');
ylabel('error (less is better)');
hold on 
plot(log(chi_manifold_rk_stats(1,:)+1),"-r",'LineWidth',3)
plot(log(chi_manifold_rk_stats(2,:)+1),"-r",'LineWidth',3)
plot(log(chi_manifold_rk_stats(3,:)+1),"-r",'LineWidth',3)
hold off

t=figure(5);
title('inliers with robust kernels');
xlabel('iteration');
ylabel('inliers (more is better)');
hold on 
plot(inliers_manifold_rk_stats(1,:),"-r",'LineWidth',3)
plot(inliers_manifold_rk_stats(2,:),"-r",'LineWidth',3)
plot(inliers_manifold_rk_stats(3,:),"-r",'LineWidth',3)
hold off
printf("press [Enter] to exit\n");
pause();

