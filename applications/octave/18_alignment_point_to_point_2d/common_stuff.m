function R=rotation2D(alpha)
  s=sin(alpha);
  c=cos(alpha);
  R=[c -s;
     s  c];
endfunction

function Rp=rotation2Dgradient(alpha)
  s=sin(alpha);
  c=cos(alpha);
  Rp=[-s -c;
      c -s];
endfunction

function T=v2t(v)
  T=eye(3);
  T(1:2,1:2)=rotation2D(v(3));
  T(1:2,3)=v(1:2);
endfunction

function P=generateRandomPoints(scale, number)
	 P=(rand(2,number)-ones(2,number)*0.5)*scale;
endfunction;

function P_new=transformPoints(P,x)
  t=x(1:2);
  theta=(x(3));
  R=rotation2D(theta);
  P_new=R*P-repmat(t,1,size(P,2));
endfunction
