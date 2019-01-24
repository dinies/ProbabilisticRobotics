source "common_stuff.m"

function [e,J]=errorAndJacobian(x,p,z)
 t=x(1:2);
 alpha=x(3);
 R=rotation2D(alpha);
 z_hat=R*p+t;
 e=z_hat-z;
 J=zeros(2,3);
 J(1:2,1:2)=eye(2);
 J(1:2,3)=rotation2Dgradient(alpha)*p;
endfunction;


function [chi, x_new]=icp2d(x,P,Z)
  chi=0; %cumulative chi2
  H=zeros(3,3);  b=zeros(3,1); %accumulators for H and b
  for(i=1:size(P,2))
     p=P(:,i); z=Z(:,i); % fetch point and measurement
     [e,J]=errorAndJacobian(x,p,z); %compute e and J for the point
     H+=J'*J;            %assemble H and B
     b+=J'*e;
     chi+=e'*e;          %update cumulative error
  endfor
  dx=-H\b;               %solve the linear system
  x_new=x+dx;            %apply update

  %normalize theta between -pi and pi
  alpha=x(3);
  s=sin(alpha); c=cos(alpha);
  x(3)=atan2(s,c);
endfunction


%example_function, call with 
%[chi_ev, x_ev]=testICP2D(5,[20 -10 pi/3]',10)

function [chi_evolution, x_evolution]=testICP2D(num_points, target_transform, iterations)
  chi_evolution=zeros(iterations,1);
  x_evolution=zeros(3,iterations);
  scale=10;
  P=generateRandomPoints(scale, num_points);
  Z=transformPoints(P,target_transform);
  x_current=zeros(3,1);
  for (i=1:iterations)
      [chi,x_current]=icp2d(x_current,P,Z);
      chi_evolution(i)=chi;
      x_evolution(:,i)=x_current;
  endfor;
endfunction

