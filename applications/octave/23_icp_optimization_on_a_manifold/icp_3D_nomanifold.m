source "../tools/utilities/geometry_helpers_3d.m"

function [e,J]=errorAndJacobian(x,p,z)
  rx=Rx(x(4));
  ry=Ry(x(5));
  rz=Rz(x(6));
  t=x(1:3);

  z_hat=rx*ry*rz*p+t;
  e=z_hat-z;
  J=zeros(3,6);
  J(1:3,1:3)=eye(3);

  rx_prime=Rx_prime(x(4));
  ry_prime=Ry_prime(x(5));
  rz_prime=Rz_prime(x(6));
  
  J(1:3,4)=rx_prime*ry*rz*p;
  J(1:3,5)=rx*ry_prime*rz*p;
  J(1:3,6)=rx*ry*rz_prime*p;
endfunction

function [x, chi_stats, num_inliers]=doICP(x_guess, P, Z, num_iterations, damping, kernel_threshold)
  x=x_guess;
  chi_stats=zeros(1,num_iterations); #ignore this for now
  num_inliers=zeros(1,num_iterations);

  #ds we perform more iterations to show that the solution does not change after one (converged)
  for (iteration=1:num_iterations)
    H=zeros(6,6);
    b=zeros(6,1);
    chi_stats(iteration)=0;
    for (i=1:size(P,2))
      [e,J] = errorAndJacobian(x, P(:,i), Z(:,i));
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
    x+=dx;
  endfor
endfunction
