%close all; clear all; clc

global t T1meas T2meas Q1 Q2

filename = 'data3.mat';
delimiterIn = ',';
headerlinesIn = 1;
z = importdata(filename,delimiterIn,headerlinesIn);

t = z(:,1);
Q1 = z(:,4);
Q2 = z(:,5);
T1meas = z(:,2);
T2meas = z(:,3);

% number of time points
ns = length(t);

load('parameters2')

p(1) = U;
p(2) = Us;
p(3) = alpha1;
p(4) = alpha2;
p(5) = tau;

disp(['U: ' num2str(U)])
disp(['Us: ' num2str(Us)])
disp(['alpha1: ' num2str(alpha1)])
disp(['alpha2: ' num2str(alpha2)])
disp(['tau: ' num2str(tau)])

% calculate model with updated parameters
Tp  = simulate(p);

% Plot results
figure(1)

subplot(3,1,1)
hold on
plot(t/60.0,T1meas,'b-','LineWidth',1)
plot(t/60.0,Tp(:,1),'r--','LineWidth',2)
ylabel('Temperature (degC)')
legend('T_1 measured','T_1 optimized')

subplot(3,1,2)
hold on
plot(t/60.0,T2meas,'b-','LineWidth',1)
plot(t/60.0,Tp(:,2),'r--','LineWidth',2)
ylabel('Temperature (degC)')
legend('T_2 measured','T_2 optimized')

subplot(3,1,3)
plot(t/60.0,Q1,'g-','LineWidth',2)
hold on
plot(t/60.0,Q2,'k--','LineWidth',2)
ylabel('Heater Output')
legend('Q_1','Q_2')

xlabel('Time (min)')

