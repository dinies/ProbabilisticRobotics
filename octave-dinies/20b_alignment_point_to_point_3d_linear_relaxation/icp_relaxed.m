                                #ICP
function [ R_est, t_est]= icp_relaxed( Z , P)

  #x= ( r11, r12, r13, r21, r22, r23 , r31, r32, r33, t1, t2, t3);
  x= zeros(1,12);

  H= zeros(12,12);
  b= zeros(12,1);


  for ( i = 1: size(P,2))
    M_i= zeros(3,9);
    M_i(1,1:3)= P(:,i)';
    M_i(2,4:6)= P(:,i)';
    M_i(3,7:9)= P(:,i)';

    r_i= zeros(9,1);
    r_i= x(1,1:9)';
    t_i= zeros(3,1);
    t_i= x(1,10:12)';

    %iniial guess at zero makes h_x_i vanish : it will be always zero
    h_x_i = M_i*r_i + t_i;

    H_i = zeros(12,12);
    H_i(1:9,1:9)= M_i'*M_i;
    H_i(1:9,10:12)= M_i';
    H_i(10:12,1:9)= M_i;
    H_i(10:12,10:12)= eye(3);

    H += H_i;

    J = zeros(3,12);
    J(1:3,1:9)= M_i;
    J(1:3,10:12);

    e_i= h_x_i- Z(:,i);
    b_i= J'* e_i;

    b += b_i;
  end

  d_x = - H \ b;
%this is only for coherence with theory since x will be always zero,unless I change my initial guess
  x_star= x' + d_x;
  R_est(1,:)= x_star(1:3,:)';
  R_est(2,:)= x_star(4:6,:)';
  R_est(3,:)= x_star(7:9,:)';
  t_est= x_star(10:12,:);
end
