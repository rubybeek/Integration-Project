%----------------------------------------------%
% State feedback controller for TCLab experiments
%----------------------------------------------%
% Different reference signals can be chosen on 
% line 95 till 116
%----------------------------------------------%

clearvars
close all

load('MIMO_para2.mat')

Ad = lin_discrete.A;
Bd = lin_discrete.B;
Cd = lin_discrete.C;
Dd = lin_discrete.D;

Ac = lin_statespace.A;
Bc = lin_statespace.B;
Cc = lin_statespace.C;
Dc = lin_statespace.D;

%% controllability test

Co = ctrb(Ad,Bd);
Ob = obsv(Ad,Cd); 

rankcheck_co = rank(Co);
rankcheck_ob = rank(Ob);

if (rankcheck_co == min(size(Co))) && (rankcheck_ob == min(size(Ob)))
    display('System is controllable and observable');
else
    display('System is NOT controllable or observable');
end

%% Poleplacement

Poles = [pole(lin_statespace)]'; %[-0.0313   -0.0313   -0.0044   -0.0134]
Poles_disc = [pole(lin_discrete)]';

P = [-0.034   -0.034   -0.0120   -0.0200];
K = place(Ac,Bc,P);

newsys = ss(Ac-Bc*K,Bc,Cc,Dc);
newsys_disc = c2d(newsys,1,'zoh'); 
Poles_dis_new = pole(newsys_disc);
MIMO = tf(newsys(1:2,1:2));

figure(1)
step(MIMO)

figure(2)
step(lin_statespace)

Kdc = dcgain(MIMO);
Kr = inv(Kdc);
   
MIMO_scaled = ss(Ac-Bc*K,Bc*Kr,Cc,Dc);

figure(3)
step(MIMO_scaled)

%% Different references
x(:,1) = [0; 0; 0; 0];

Tamb = 21;
%switching ref, different per sensor
% r_input1(1,:) = [repmat(19,1,600),repmat(14,1,500),repmat(22,1,300),repmat(22,1,600)]; %40 deg
% r_input1(2,:) = [repmat(10,1,500),repmat(14,1,300),repmat(14,1,600),repmat(19,1,600)]; %40 deg
% r = [r_input1];

% r_input1(1,:) = [repmat(19,1,600)]; %40 deg
% r_input1(2,:) = [repmat(19,1,600)]; %40 deg
% r = [r_input1];

% r_input1 = zeros(2,1500);
% r_input1(1,:) = [repmat(19,1,1500)]; %40 deg
% r_input1(2,:) = [repmat(19,1,300),repmat(24,1,300),repmat(14,1,300),repmat(24,1,300),repmat(14,1,300)]; %40 deg
% r = [r_input1];

r_input1 = repmat(9,2,600); %40 deg
r_input2 = repmat(19,2,600); %35 deg
r = [r_input1 r_input2];

%% simulation
for i = 1:1200
    u(:,i) = Kr*r(:,i) - K*x(:,i);
 
    if u(1,i) <= 0
        u(1,i) = 0;
    end
    if u(2,i) <= 0
        u(2,i) = 0;
    end
    x(:,i+1) = Ad*x(:,i) + Bd*u(:,i);  
end
%% Figures for simulation
figure()
subplot(2,1,1)
plot(linspace(1,length(x),length(x)),x(3,:)+Tamb,'LineWidth',1)
hold on
plot(linspace(1,length(x),length(x)),x(4,:)+Tamb,'LineWidth',1)
plot(linspace(1,length(r),length(r)),r(1,:)+Tamb,'k')
%plot(linspace(1,length(r),length(r)),r(2,:)+Tamb,'k
legend('T1','T2');
ylim([20 45])
xlim([0 500])

subplot(2,1,2)
plot(linspace(1,length(u),length(u)),u(1,:),'LineWidth',1)
hold on
plot(linspace(1,length(u),length(u)),u(2,:),'LineWidth',1)
legend('U1','U2');
xlim([0 500])