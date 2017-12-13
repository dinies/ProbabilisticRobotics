#does ICP and returns rotation (R) and translation (t).
#X and P are  equally sized and contain the correspoinding point in columns
function [R,t]=icp(X,P)
  xNum=size(X)(2);
  pNum=size(P)(2);
  R=eye(1);
  t=zeros(1,3);
  if (xNum != pNum)
    return;
  endif;
  
  #compute the means
  xm=(1./xNum)*X*ones(xNum,1);
  pm=(1./pNum)*P*ones(pNum,1);
  
  #subtract the means
  Xm=X-xm*ones(1,xNum);
  Pm=P-pm*ones(1,pNum);
  
  #compute A
  A=Xm*Pm';
  [U,s,V]=svd(A);
  R=U*V';
  
  #compute t
  t=xm-R*pm;
end;
