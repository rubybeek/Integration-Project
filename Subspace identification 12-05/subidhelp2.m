function [Bt,Dt,x0t,theta,Phi_N]=subidhelp2(y,u,At,Ct,N,n,yold)

L = 2;

% Extract matrices Bt and Dt following section 9.2.4.3
[ks,~] = size(Ct*At); 
[~,Lu] = size(u);

Phi_N = zeros(N*ks,n + (Lu*n) + (Lu*L)); 

for k = 1:N

    clear Phi_1 Phi_2 Phi_3 
    
    Phi_2 = 0;

    Phi_1 = Ct*(At^k);

    
        for tau = 0:(k-1)
 
            Phi_2 = Phi_2 + kron(u(tau+1,:)',Ct*(At^(k-tau-1)));
        end
        
        Phi_3 = kron(u(k,:)',eye(L));

        Phi_N(1 + ks*(k-1):ks*k,:) = [Phi_1 Phi_2 Phi_3];
end

%Lu = 2;

Phi_N_T = Phi_N';

theta = ((1/N)*Phi_N_T*Phi_N)\((1/N)*Phi_N_T.*yold(:,1));
    
x0t = theta(1:n,1);
    
Bt = reshape(theta((n+1):(n+(n*Lu))),n,Lu);
    
Dt = reshape(theta(n+(n*Lu)+1:(n+(n*Lu)+ (Lu*L))),L,Lu);
end