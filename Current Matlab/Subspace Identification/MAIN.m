clear all 
close all
clearvars

load('data3.mat')
data1 = data;
load('data4.mat')
data2 = data;

%% preprocessing 
% Tamb = min(data(1,1),data(1,2));
% data(:,1:2) = data(:,1:2) - Tamb;
Tamb = min(min(data1(1,2),data1(1,3)),min(data2(1,2),data2(1,3)));
data1(:,2:3) = data1(:,2:3) - Tamb;
data2(:,2:3) = data2(:,2:3) - Tamb;

%% Create datasets for Compare function
% data for identification
dataset1 = iddata(data1(:,2:3),data1(:,4:5),1);
dataset1.InputName  = {'Input Sensor 1';'Input Sensor 2'};
dataset1.OutputName = {'Output Sensor 1';'Output Sensor 2'};

% validation data
dataset2 = iddata(data2(:,2:3),data2(:,4:5),1);
dataset2.InputName  = {'Input Sensor 1';'Input Sensor 2'};
dataset2.OutputName = {'Output Sensor 1';'Output Sensor 2'};

%% subspace identification Verhaegen
yold = data1(:,2:3);
uold = data1(:,4:5);
N = length(yold);

y = zeros(2*N,1);
u = zeros(2*N,1);

for i = 1:N
    y(2*i-1:2*i) = yold(i,:)';
    u(2*i-1:2*i) = uold(i,:)';
end

%% Subspace identificaton using Filtering book / RQ factorization / Verhaegen

%for i = 1:9
n = 10; %n = 1+i;
s = n*4;

[At, Bt, Ct, Dt, x0t, S, theta,Phi_N] = mysubid(y,u,s,n,yold,N);

Ts = 1;
sysd = ss(At,Bt,Ct,Dt,Ts);
sysc = d2c(sysd);

figure()
compare(dataset2,sysd);
title('Subspace identification using RQ Factorization')
%end

%% Right choices
ssest_model = ssest(dataset1,6);
ssest_model = c2d(ssest_model,1);
n4sid_model = n4sid(dataset1,5);
Verhaegen_model = sysd;
 
figure(1)
compare(dataset2,ssest_model,n4sid_model,Verhaegen_model,'LineWidth',1);

%save('Subspacemodels','ssest_model','n4sid_model','Verhaegen_model')

%% Subspace identification using ssest
% MIMO2 = ssest(dataset1,2);
% MIMO2 = c2d(MIMO2,1);
% MIMO3 = ssest(dataset1,3);
% MIMO3 = c2d(MIMO3,1);
% MIMO4 = ssest(dataset1,4);
% MIMO4 = c2d(MIMO4,1);
% MIMO5 = ssest(dataset1,5);
% MIMO5 = c2d(MIMO5,1);
% MIMO6 = ssest(dataset1,6);
% MIMO6 = c2d(MIMO6,1);
% MIMO7 = ssest(dataset1,7);
% MIMO7 = c2d(MIMO7,1);
% MIMO8 = ssest(dataset1,8);
% MIMO8 = c2d(MIMO8,1);
% MIMO9 = ssest(dataset1,9);
% MIMO9 = c2d(MIM09,1);
% MIMO10 = ssest(dataset1,10);
% MIMO10 = c2d(MIMO10,1);
% 
% figure(4)
% compare(dataset2,MIMO1);
% title('Subspace identification using ssest, 2nd order')
% 
% figure(5)
% compare(dataset2,MIMO2);
% title('Subspace identification using ssest, 3th order')
% 
% figure(6)
% compare(dataset2,MIMO3);
% title('Subspace identification using ssest, 4th order')
% 
% figure(7)
% compare(dataset2,MIMO4);
% title('Subspace identification using ssest, 5th order')
% 
% figure(8)
% compare(dataset2,MIMO5);
% title('Subspace identification using ssest, 6th order')
% 
% figure(9)
% compare(dataset2,MIMO6);
% title('Subspace identification using ssest, 7th order')
% 
% figure(10)
% compare(dataset2,MIMO7);
% title('Subspace identification using ssest, 8th order')
% 
% figure(11)
% compare(dataset2,MIMO8);
% title('Subspace identification using ssest, 9th order')
% 
% figure(12)
% compare(dataset2,MIMO9);
% title('Subspace identification using ssest, 10th order')

%% Subspace identification using N4SID
% MIMO10 = n4sid(dataset1,2);
% MIMO11 = n4sid(dataset1,3);
% MIMO12 = n4sid(dataset1,4);
% MIMO13 = n4sid(dataset1,5);
% MIMO14 = n4sid(dataset1,6);
% MIMO15 = n4sid(dataset1,7);
% MIMO16 = n4sid(dataset1,8);
% MIMO17 = n4sid(dataset1,9);
% MIMO18 = n4sid(dataset1,10);
% 
% figure(13)
% compare(dataset2,MIMO10);
% title('Subspace identification using n4sid, 2nd order')
% 
% figure(14)
% compare(dataset2,MIMO11);
% title('Subspace identification using n4sid, 3th order')
% 
% figure(15)
% compare(dataset2,MIMO12);
% title('Subspace identification using n4sid, 4th order')
% 
% figure(16)
% compare(dataset2,MIMO13);
% title('Subspace identification using n4sid, 5th order')
% 
% figure(17)
% compare(dataset2,MIMO14);
% title('Subspace identification using n4sid, 6th order')
% 
% figure(18)
% compare(dataset2,MIMO15);
% title('Subspace identification using n4sid, 7th order')
% 
% figure(19)
% compare(dataset2,MIMO16);
% title('Subspace identification using n4sid, 8th order')
% 
% figure(20)
% compare(dataset2,MIMO17);
% title('Subspace identification using n4sid, 9th order')
% 
% figure(21)
% compare(dataset2,MIMO18);
% title('Subspace identification using n4sid, 10th order')