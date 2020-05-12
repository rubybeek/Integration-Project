clear all 
close all
clearvars

load('data.1.mat')

yold = data(:,1:2);
uold = data(:,3:4);
N = length(yold);

y = zeros(2*399,1);
u = zeros(2*399,1);

n = 2;
s = n*4;
 
for i = 1:N
    y(2*i-1:2*i) = yold(i,:)';
    u(2*i-1:2*i) = uold(i,:)';
end

%%
%Y = zeros(2*s,N-s); 
% for i = 1:N-s+1
%    Y(:,i) = y(2*i-1 : 2*s -2 + 2*i); 
%    U(:,i) = u(2*i-1 : 2*s -2 + 2*i); 
% end

[At, Bt, Ct, Dt, x0t, S] = mysubid(y,u,s,n);






