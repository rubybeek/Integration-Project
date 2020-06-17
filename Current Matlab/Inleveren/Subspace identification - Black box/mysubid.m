function [At, Bt, Ct, Dt, x0t, S, theta, Phi_N] = mysubid(y,u,s,n,yold,N)

if n>=s
    disp('s is not chosen bigger than n');
end

for i = 1:N-s+1
   Y(:,i) = y(2*i-1 : 2*s -2 + 2*i); 
   U(:,i) = u(2*i-1 : 2*s -2 + 2*i); 
end

r = triu(qr([U;Y]'))';
R22 = r((2*s+1:end),(2*s+1:4*s));

[uu,S,~]=svd(R22); %singular values of R22 are the same as those from Y*PI

Ct = uu(1:2,1:n);
At = pinv(uu((1:(s-1)*2),1:n)) * uu(3:2*s,1:n);

[Bt,Dt,x0t,theta,Phi_N]=subidhelp2(y,u,At,Ct,N,n,yold);
end
