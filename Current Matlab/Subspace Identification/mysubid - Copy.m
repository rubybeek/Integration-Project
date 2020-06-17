function [At, Bt, Ct, Dt, x0t, S, theta, Phi_N] = mysubid(y,u,s,n,yold,N)

if n>=s
    disp('s is not chosen bigger than n');
end

%N=399; %length(y/2);

% Y = hankel(y(1:s),y(s:N));
% U = hankel(u(1:s),u(s:N));

for i = 1:N-s+1
   Y(:,i) = y(2*i-1 : 2*s -2 + 2*i); 
   U(:,i) = u(2*i-1 : 2*s -2 + 2*i); 
end

%[Q,R]=qr([U;Y]');
r = triu(qr([U;Y]'))';
R22 = r((2*s+1:end),(2*s+1:4*s));
%R22=r((s+1:end),(s+1:2*s)); % as for R11 an R22 have dimensions sxs
%Q2 = Q((s+1:2*s),:);

[uu,S,~]=svd(R22); %singular values of R22 are the same as those from Y*PI
%figure
%semilogy(diag(S),'xr');

%Ct = uu(1,1:s-(s-n));
%At = uu(1:s-(s-n),:)/uu(2:s-(s-n-1),:);
Ct = uu(1:2,1:n);
At = pinv(uu((1:(s-1)*2),1:n)) * uu(3:2*s,1:n);

% Bt=1;
% Dt=1;
% x0t=1;
[Bt,Dt,x0t,theta,Phi_N]=subidhelp2(y,u,At,Ct,N,n,yold);
end
