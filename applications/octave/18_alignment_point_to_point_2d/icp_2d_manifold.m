source "common_stuff.m"

function [e,J]=errorAndJacobianManifold(X,p,z)
 t=X(1:2,3);
 R=X(1:2,1:2);
 z_hat=R*p+t;
 e=z_hat-z;

 J=zeros(2,3);
 J(1:2,1:2)=eye(2);
 J(1:2,3)=[-z_hat(2),
	   z_hat(1)]';
endfunction;

function [chi,X]=icp2dManifold(X,P,Z)
  chi=0; %cumulative chi2
  H=zeros(3,3);  b=zeros(3,1); %accumulators for H and b
  for(i=1:size(P,2))
     p=P(:,i); z=Z(:,i); % fetch point and measurement
     [e,J]=errorAndJacobianManifold(X,p,z); %compute e and J for the point
     H+=J'*J;            %assemble H and B
     b+=J'*e;
     chi+=e'*e;          %update cumulative error
  endfor
  dx=-H\b;               %solve the linear system
  X=v2t(dx)*X;
endfunction

function [chi_evolution, x_evolution]=testICP2DManifold(num_points, target_transform, iterations)
  chi_evolution=zeros(iterations,1);
  scale=10;
  P=generateRandomPoints(scale, num_points);
  Z=transformPoints(P,target_transform);
  X_current=eye(3);
  for (i=1:iterations)
      [chi, X_current]=icp2dManifold(X_current,P,Z);
      chi_evolution(i)=chi;
  endfor;
endfunction


