function [Bt,Dt,x0t]=subidhelp(y,u,At,Ct)
% SUBIDHELP   Subspace identification for a SISO system, help
%   [B,D,x0]=subidhelp(y,u,A,C) returns the state space matrices B and D
%   of a discrete-time LTI SISO system from its input vector u and its
%   output vector y, and from the estimated matrices A and C. Also the 
%   initial state x0 is returned.

  
  [nmax,a1]=size(y);
  [nmax2,a2]=size(u);
  
  if a1>1
    error('The parameter y must have a single column.');
  end
  
  if a2>1
    error('The parameter u must have a single column.') ;
  end
  
  if nmax2 ~= nmax
    error('The vectors u and y must have the same size.');
  end
  
  [c1,c2]=size(Ct);
  [a1,a2]=size(At);

  if c1 ~= 1
    error('C must be a row matrix.');
  end
  
  if a1~=a2
      error('A must be square.');
  end
  
  if c2~=a1
      error('The dimensions of A and C are not consistent.')
  end

  n=a1;

  PSI=[];
  for k=0:nmax-1
    LL=zeros(1,n);
    for tau=0:k-1
        LL=LL+u(tau+1)*Ct*At^(k-1-tau);
    end
    PSI=[PSI; Ct*At^k LL u(k+1)'];
  end
  
  teta=pinv(PSI)*y;
 
  x0t=[];
  for k=1:n
     x0t=[x0t ; teta(1)];
     teta=teta(2:end);
  end
  
  Bt=[];
  for k=1:n
     Bt=[Bt ; teta(1)];
     teta=teta(2:end);
  end

  Dt=teta;
  
end