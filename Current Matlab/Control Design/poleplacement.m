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

%P = [pole(lin_statespace)]';
P = [-0.0626 -.0626 -0.009 -0.027];
K = place(Ac,Bc,P);

newsys = ss(Ac-Bc*K,Bc,Cc,Dc);

MIMO = tf(newsys(1:2,1:2));

figure(1)
step(MIMO)

% figure(2)
% step(lin_statespace)

Kdc = dcgain(MIMO);
Kr = inv(Kdc);

MIMO_scaled = ss(Ac-Bc*K,Bc*Kr,Cc,Dc);

figure(3)
step(MIMO_scaled)

%% design observer 

[Kest,L,P2] = kalman(MIMO_scaled,[],[],[]);

Ts = 10;

kalmansys_discr = c2d(Kest,Ts);

MIMO_scaleddiscr = c2d(MIMO_scaled,Ts);

figure(4)
bode(MIMO_scaled)

figure(5)
bode(lin_statespace)

figure(6)
step(MIMO_scaleddiscr)

%%

Ad_sc = MIMO_scaleddiscr.A
Tamb = 21;
R
tclab;
initial_state = [T1C()-Tamb T2C()-Tamb T1C()-Tamb T2C()-Tamb];

ht1 = 0;
ht2 = 0;
initial_input = [h1(ht1) h2(ht2)];

% for i = 1:50  %50 x 10 sec
%     
%     x(i+1) = A
%     
% end










