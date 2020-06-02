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

rankcheck = rank(Co);

if rankcheck == min(size(Co))
    display('System is controllable');
else
    display('System is NOT controllable');
end


%%

% P = [pole(lin_statespace)]';
% %P = [-0.0313   -0.0313   -0.0044   -0.0134];
% P = [-0.01   -0.01   -0.004   -0.013 ];
% % P = [-0.0626 -.0626 -0.009 -0.027];
% K = place(Ac,Bc,P);

R = 1E3;
Q = eye(4);

%[P,L,K] = dare(Ac,Bc,Q,R);
[K,P,E] = lqr(Ac,Bc,Q,R);


newsys = ss(Ac-Bc*K,Bc,Cc,Dc);

MIMO = tf(newsys(1:2,1:2));

figure(1)
step(MIMO)

figure(2)
step(lin_statespace)

Kdc = dcgain(MIMO(1,1));
%Kdc = dcgain(lin_discrete);
%Kr = abs(inv(Kdc));
Kr = inv(Kdc);
%Kr = [1/Kdc(1,1) 1/Kdc(1,2);1/Kdc(2,1) 1/Kdc(2,2)]
    
%MIMO_scaled = ss(Ac-Bc*K,Bc*Kr,Cc,Dc);
MIMO_scaled = ss(Ac,Bc*Kr,Cc,Dc);

figure(3)
step(MIMO_scaled)

%% design observer 

%[Kest,L,P2] = kalman(MIMO_scaled,[],[],[]);

Ts = 1;

%kalmansys_discr = c2d(Kest,Ts);
MIMO_scaleddiscr = c2d(MIMO_scaled,Ts);

[Kest,L,P2] = kalman(MIMO_scaleddiscr,[],[],[]);
% ;[Kest,S,e] = lqr(MIMO_scaleddiscr,Q,R,N);

figure(4)
bode(MIMO_scaled)

figure(5)
bode(lin_statespace)

figure(6)
step(MIMO_scaleddiscr)

%%

Ad_sc = MIMO_scaleddiscr.A;
Bd_sc = MIMO_scaleddiscr.B;
Cd_sc = MIMO_scaleddiscr.C;

Tamb = 21;
r = [20;20];
tclab;
x(:,1) = [T1C()-Tamb; T2C()-Tamb; T1C()-Tamb; T2C()-Tamb];

h1s = [];
h2s = [];
t1s = [];
t2s = [];

ht1 = 0;
ht2 = 0;
h1(ht1);
h2(ht2);

for i = 1:50  %50 x 10 sec   
    u(:,i) = Kr*r - K*x(:,i);
    ht1 = u(1,i);
    ht2 = u(2,i);
    h1(ht1);
    h2(ht2);
    t1 = T1C();
    t2 = T2C();
    y = [t1-Tamb; t2-Tamb];
    y_hat(:,i) = Cd_sc*x(:,i);
    %y = y_hat(:,1);
    x(:,i+1) = Ad_sc*x(:,i) + Bd_sc*u(:,i)+ L*(y - y_hat(:,1));
    
    h1s = [h1s,ht1];
    h2s = [h2s,ht2];
    t1s = [t1s,t1];
    t2s = [t2s,t2];
    
    n = length(t1s);
    time = linspace(0,(n+1),n);
    
    clf
    subplot(2,1,1)
    plot(time,t1s,'b.','MarkerSize',10);
    hold on
    plot(time,t2s,'r.','MarkerSize',10);
    ylabel('Temperature (degC)')
    legend('Temperature 1','Temperature 2','Location','NorthWest')
    
    subplot(2,1,2)
    plot(time,h1s,'b-','LineWidth',2);
    hold on
    plot(time,h2s,'r--','LineWidth',2);
    ylabel('Heater (0-5.5 V)')
    xlabel('Time (sec)')
    legend('Heater 1','Heater 2','Location','NorthWest')
    drawnow;
    %t = toc;

    pause(Ts)
end

h1(0);
h2(0);








