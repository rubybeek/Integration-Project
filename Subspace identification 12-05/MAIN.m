clear all 
close all
clearvars

load('data.1.mat')
datasmooth = smoothdata(data(:,1:2),'sgolay',60);

figure(1)
subplot(2,1,1)
plot(datasmooth(:,1))
hold on
plot(data(:,1))
ylabel('Temperature (degC)')
title('Smoothened Data Sensor 1')

subplot(2,1,2)
plot(datasmooth(:,2))
hold on
plot(data(:,2))
title('Smoothened Data Sensor 2')
ylabel('Temperature (degC)')
xlabel('Time (sec)')

dataset = iddata(datasmooth(:,1:2),data(:,3:4),1);
dataset.InputName  = {'Input Sensor 1';'Input Sensor 2'};
dataset.OutputName = {'Output Sensor 1';'Output Sensor 2'};

%% subspace identification 
yold = datasmooth(:,1:2);
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

[At, Bt, Ct, Dt, x0t, S, theta,Phi_N] = mysubid(y,u,s,n,yold);

Ts = 1;
sysd = ss(At,Bt,Ct,Dt,Ts);
sysc = d2c(sysd);

figure(2)
compare(dataset,sysd);
title('Subspace identification using own script')

%% Subspace identification using ssest
MIMO = ssest(dataset,2);

figure(3)
compare(dataset,MIMO);
title('Subspace identification using ssest')

%% Subspace identification using N4SID
MIMO2 = n4sid(dataset,2);

figure(4)
compare(dataset,MIMO2);
title('Subspace identification using N4SID')
