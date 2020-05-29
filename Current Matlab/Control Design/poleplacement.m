clearvars
close all

load('Linearization ss model discrete.mat');
load('Linearization ss model continuous.mat');

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


%% Pole-placement

% Check open loop poles

P_OL = pole(lin_statespace);

% Make closed loop
figure(1)
step(lin_statespace)

% Solve for gain
Kdc = dcgain(lin_statespace);
Kr = inv(Kdc);

% Create scaled input CL system
syscl_scaled = ss(Ac,Bc*Kr,Cc,Dc);

figure(2)
step(syscl_scaled)


%%

figure(1)
step(lin_discrete)

figure(2)
pzmap(lin_discrete)

figure(3)
pzmap(lin_statespace)

p = pole(lin_statespace);
%K = place(Ac,Bc,[-5+3*j -5-3*j -10 -15]);
K = place(Ac,Bc,p)
%closedloop_cont = feedback(G,K);
sys = ss(Ac-Bc*K,Bc,Cc,Dc);
[num,den] = ss2tf(Ac-Bc*K,Bc,Cc,Dc,1);
%G = tf(num,den)
MIMO = tf(sys(1:2,1:2));
% 
% figure(4)
% step(closedloop_cont)










