%-----------------------------------------%
% Subspace Identification
%-----------------------------------------%

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

%for i = 1:12 
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
ssest_model1 = ssest(dataset1,6);
ssest_n4sid = c2d(ssest_model1,1);
ssest_model2 = ssest(dataset1,8,'n4Weight','MOESP');
ssest_moesp = c2d(ssest_model2,1);
Verhaegen_model = sysd;

%%
figure(3)
compare(dataset2,ssest_n4sid,ssest_moesp,Verhaegen_model);
 
%% saving subspace
save('Subspacemodels','ssest_n4sid','ssest_moesp','Verhaegen_model')

%% Subspace identification using ssest with MOESP - order estimation
% MIMO2 = ssest(dataset1,2,'n4Weight','MOESP');
% MIMO2 = c2d(MIMO2,1);
% MIMO3 = ssest(dataset1,3,'n4Weight','MOESP');
% MIMO3 = c2d(MIMO3,1);
% MIMO4 = ssest(dataset1,4,'n4Weight','MOESP');
% MIMO4 = c2d(MIMO4,1);
% MIMO5 = ssest(dataset1,5,'n4Weight','MOESP');
% MIMO5 = c2d(MIMO5,1);
% MIMO6 = ssest(dataset1,6,'n4Weight','MOESP');
% MIMO6 = c2d(MIMO6,1);
% MIMO7 = ssest(dataset1,7,'n4Weight','MOESP');
% MIMO7 = c2d(MIMO7,1);
% MIMO8 = ssest(dataset1,8,'n4Weight','MOESP');
% MIMO8 = c2d(MIMO8,1);
% MIMO9 = ssest(dataset1,9,'n4Weight','MOESP');
% MIMO9 = c2d(MIM09,1);
% MIMO10 = ssest(dataset1,10,'n4Weight','MOESP');
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

%% Subspace identification using ssest using N4SID - order estimation
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