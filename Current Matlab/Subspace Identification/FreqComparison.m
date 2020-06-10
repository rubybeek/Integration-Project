clear all 
close all 
clc

load('MIMO_para2.mat')

%% Sensitivity Function
%tf_parest = ss2tf(lin_statespace.A,lin_statespace.B,lin_statespace.C,lin_statespace.D,2);
tf_parest = tf(lin_statespace);
S = inv(eye(2)+tf_parest);
T = tf_parest*inv(eye(2)+tf_parest);

figure(1)
bode(S(1,1))
hold on
bode(T(1,1))
bode(tf_parest(1,1))
legend('S','T')

figure(2)
bode(S(2,2))
hold on
bode(T(2,2))
bode(tf_parest(2,2))
legend('S','T')