function[At, Bt, Ct, Dt, x0t, S]=mysubid(y ,u ,s ,n)

[Ny,l] = size(y);

N = Ny-s+1;

Yhank = hankel(y(1:s),y(s:N+s-1));
Uhank = hankel(u(1:s),u(s:N+s-1));

%RQ
r = triu(qr([Uhank;Yhank]'))';

R22 = r(s+1:end,s+1:2*s);  %size of R22 from page 304

[U, Sigma, ~]=svd(R22);

%Projection
% ProjU = eye(size(Uhank,2))-(Uhank')*((Uhank*(Uhank'))^-1)*Uhank;
% ProjY = Uhank*ProjU;

% [U, Sigma, ~]=svd(ProjY);

At = U(1:(s-1)*l,1:n)\U(l+1:s*l,1:n);

Ct = U(1:l,1:n);

S = diag(Sigma);

[Bt,Dt,x0t] = subidhelp(y,u,At,Ct);

end



