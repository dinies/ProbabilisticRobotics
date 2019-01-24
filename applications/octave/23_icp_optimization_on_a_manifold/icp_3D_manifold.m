source "../tools/utilities/geometry_helpers_3d.m"

function [e,J]=errorAndJacobianManifold(X,p,z)
   z_hat=X(1:3,1:3)*p+X(1:3,4); #prediction
   e=z_hat-z;
   J=zeros(3,6);
   J(1:3,1:3)=eye(3);
   J(1:3,4:6)=-skew(z_hat);
endfunction

function [X, chi_stats, num_inliers]=doICPManifold(X_guess, P, Z, num_iterations, damping, kernel_threshold)
  X=X_guess;
  chi_stats=zeros(1,num_iterations);
  num_inliers=zeros(1,num_iterations);
  for (iteration=1:num_iterations)
    H=zeros(6,6);
    b=zeros(6,1);
    chi_stats(iteration)=0;
    for (i=1:size(P,2))
      [e,J] = errorAndJacobianManifold(X, P(:,i), Z(:,i));
      chi=e'*e;
      if (chi>kernel_threshold)
	 e*=sqrt(kernel_threshold/chi);
	 chi=kernel_threshold;
      else
	num_inliers(iteration)++;
      endif;
      chi_stats(iteration)+=chi;
      H+=J'*J;
      b+=J'*e;
    endfor
    H+=eye(6)*damping;
    dx=-H\b;
    X=v2t(dx)*X;
  endfor
endfunction
