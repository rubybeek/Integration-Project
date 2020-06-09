clear all 
close all
clearvars

load('data3.mat')
data1 = data;
load('data4.mat')
data2 = data;
%load('data.1.mat')

%% preprocessing 
% Tamb = min(data(1,1),data(1,2));
% data(:,1:2) = data(:,1:2) - Tamb;
Tamb = min(min(data1(1,2),data1(1,3)),min(data2(1,2),data2(1,3)));
data1(:,2:3) = data1(:,2:3) - Tamb;
data2(:,2:3) = data2(:,2:3) - Tamb;
%% Smoothdata optional
% datasmooth = smoothdata(data(:,1:2),'sgolay',60);
% 
% figure(1)
% subplot(2,1,1)
% plot(datasmooth(:,1))
% hold on
% plot(data(:,1))
% ylabel('Temperature (degC)')
% title('Smoothened Data Sensor 1')
% 
% subplot(2,1,2)
% plot(datasmooth(:,2))
% hold on
% plot(data(:,2))
% title('Smoothened Data Sensor 2')
% ylabel('Temperature (degC)')
% xlabel('Time (sec)')

dataset1 = iddata(data1(:,2:3),data1(:,4:5),1);
dataset1.InputName  = {'Input Sensor 1';'Input Sensor 2'};
dataset1.OutputName = {'Output Sensor 1';'Output Sensor 2'};

dataset2 = iddata(data2(:,2:3),data2(:,4:5),1);
dataset2.InputName  = {'Input Sensor 1';'Input Sensor 2'};
dataset2.OutputName = {'Output Sensor 1';'Output Sensor 2'};

%% Subspace identification using ssest
MIMO1 = ssest([dataset1, dataset2],2);
MIMO2 = ssest([dataset1, dataset2],4);

figure(2)
compare(dataset1,MIMO1);
title('Subspace identification using ssest, 2nd order')

figure(3)
compare(dataset1,MIMO2);
title('Subspace identification using ssest, 4th order')

figure(4)
compare(dataset2,MIMO1);
title('Subspace identification using ssest, 2nd order')

figure(5)
compare(dataset2,MIMO2);
title('Subspace identification using ssest, 4th order')

%% subspace identification 
yold = data(:,2:3);
uold = data(:,4:5);
N = length(yold);

y = zeros(2*N,1);
u = zeros(2*N,1);

n = 4;
s = n*4;
 
for i = 1:N
    y(2*i-1:2*i) = yold(i,:)';
    u(2*i-1:2*i) = uold(i,:)';
end

%% Subspace identificaton using Filtering book / RQ factorization
[At, Bt, Ct, Dt, x0t, S, theta,Phi_N] = mysubid(y,u,s,n,yold,N);

Ts = 1;
sysd = ss(At,Bt,Ct,Dt,Ts);
sysc = d2c(sysd);

figure(1)
compare(dataset1,sysd);
title('Subspace identification using RQ Factorization')

%% Subspace identification using ssest
MIMO1 = ssest(dataset1,2);
MIMO2 = ssest(dataset1,4);

figure(2)
compare(dataset1,MIMO1);
title('Subspace identification using ssest, 2nd order')

figure(3)
compare(dataset1,MIMO2);
title('Subspace identification using ssest, 4th order')

%% Subspace identification using N4SID
MIMO3 = n4sid(dataset1,2);
MIMO4 = n4sid(dataset1,4);

figure(4)
compare(dataset1,MIMO3);
title('Subspace identification using N4SID, 2nd order')

figure(5)
compare(dataset1,MIMO4);
title('Subspace identification using N4SID, 4th order')
