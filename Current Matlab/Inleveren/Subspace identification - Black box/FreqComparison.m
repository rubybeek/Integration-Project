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
compare(datafft,lin_statespace,Verhaegen_model,ssest_n4sid,ssest_moesp)

%%
tf_parest = tf(lin_discrete);
tf_moesp = tf(ssest_moesp);
tf_n4sid = tf(ssest_n4sid);
tf_Verhaegen = tf(Verhaegen_model);
% 
% figure()
% bode(tf_parest)
% hold on
% bode(tf_ssest)
% bode(tf_n4sid)
% bode(tf_Verhaegen)
% legend('para','ssest','n4sid','ver')
 
%% PZmap
figure()
pzmap(tf_parest)
figure()
pzmap(tf_moesp)
figure()
pzmap(tf_n4sid)
figure()
pzmap(tf_Verhaegen)