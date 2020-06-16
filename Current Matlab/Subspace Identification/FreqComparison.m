clear all 
close all 
clc

load('MIMO_para2.mat')
load('Subspacemodels.mat')

load('data3.mat')
data1 = data;
load('data4.mat')
data2 = data;

%% preprocessing 
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

%% Sensitivity Function
%tf_parest = ss2tf(lin_statespace.A,lin_statespace.B,lin_statespace.C,lin_statespace.D,2);
% tf_parest = tf(lin_statespace);
% S = inv(eye(2)+tf_parest);
% T = tf_parest*inv(eye(2)+tf_parest);
% 
% figure(1)
% bode(S(1,1))
% hold on
% bode(T(1,1))
% bode(tf_parest(1,1))
% legend('S','T')
% 
% figure(2)
% bode(S(2,2))
% hold on
% bode(T(2,2))
% bode(tf_parest(2,2))
% legend('S','T')

%% residual analysis
% figure(1)
% resid(dataset2,lin_statespace)
% figure(2)
% resid(dataset2,Verhaegen_model)
% figure(3)
% resid(dataset2,ssest_model)
% figure(4)
% resid(dataset2,n4sid_model)
% 
% figure(5)
% resid(dataset2,lin_statespace,ssest_model,n4sid_model,Verhaegen_model)

%% compare Freq data
datafft = fft(dataset2);

figure(6)
compare(datafft,lin_statespace,Verhaegen_model,ssest_model,n4sid_model)
figure(5)
compare(datafft,lin_statespace)

% figure(4)
% compare(datafft,ssest_model)
% figure(3)
% compare(datafft,n4sid_model)
% figure(2)
% compare(datafft,Verhaegen_model)

%%
tf_parest = tf(lin_statespace);
tf_ssest = tf(ssest_model);
tf_n4sid = tf(n4sid_model);
tf_Verhaegen = tf(Verhaegen_model);

figure()
bode(tf_parest)
hold on
bode(tf_ssest)
bode(tf_n4sid)
bode(tf_Verhaegen)
legend('para','ssest','n4sid','ver')

%%

figure()
step(tf_parest)

figure()
step(tf_ssest)
figure()
step(tf_n4sid)
figure()
step(tf_Verhaegen)
legend('para','ssest','n4sid','ver')

%%

figure()
pzmap(tf_parest)

figure()
pzmap(tf_ssest)
figure()
pzmap(tf_n4sid)
figure()
pzmap(tf_Verhaegen)