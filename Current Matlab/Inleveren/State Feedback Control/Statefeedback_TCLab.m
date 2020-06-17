%----------------------------------------------%
% State feedback controller for TCLab experiments
%----------------------------------------------%
% Different reference signals can be chosen on 
% line 92 till 119
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

Ts = 1;

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

%% Observer Design
Qkal = [40 0; 0 40];
Rkal =eye(2)*(1E-2)*5;
Nkal =eye(2);

[KEST,L,Pkal] = kalman(lin_discrete,Qkal,Rkal,Nkal);

%% Observer Testing
% load('data3.mat')
% uobs = data(:,3:4)';
% x_data = data(:,1:2)';
% xobs(:,1) = [21.8475; 20.8700; 21.8475; 20.8700];
% 
% for i = 1:length(data)-1
%     yobs = x_data(:,i);
%     y_hatobs = Cd*xobs(:,i);
%     xobs(:,i+1) = Ad*xobs(:,i) + Bd*uobs(:,i)+ L*(yobs - y_hatobs);
% end
% 
% figure
% plot(linspace(1,length(data),length(data)),x_data(1,:))
% hold on
% plot(linspace(1,length(data),length(data)),xobs(3,:))
% plot(linspace(1,length(data),length(data)),x_data(2,:))
% plot(linspace(1,length(data),length(data)),xobs(4,:))
% legend('measured T1','Observed T1','measured T2','Observed T2')

%% Different references
x(:,1) = [0; 0; 0; 0];
Tamb = 21;

% % switching ref, different per sensor
% r_input1(1,:) = [repmat(19,1,600),repmat(14,1,500),repmat(22,1,300),repmat(22,1,600)]; %40 deg
% r_input1(2,:) = [repmat(10,1,500),repmat(14,1,300),repmat(14,1,600),repmat(19,1,600)]; %40 deg
% r = [r_input1];

% % Constant ref, 40 degrees
% r_input1(1,:) = [repmat(19,1,600)]; %40 deg
% r_input1(2,:) = [repmat(19,1,600)]; %40 deg
% r = [r_input1];

% % sensor 1 constant ref, sensor 2 varying ref
% r_input1 = zeros(2,1500);
% r_input1(1,:) = [repmat(19,1,1500)]; %40 deg
% r_input1(2,:) = [repmat(19,1,300),repmat(24,1,300),repmat(14,1,300),repmat(24,1,300),repmat(14,1,300)]; %40 deg
% r = [r_input1];

% % sinewave reference
% load('referencesin2.mat')
% %ref = [ref ref]'-Tamb; %average at 30 deg
% r = [ref ref]'-Tamb+5; %average at 35 deg

% % switching ref, same per sensor
r_input1 = repmat(9,2,600); %40 deg
r_input2 = repmat(19,2,600); %35 deg
r = [r_input1 r_input2];

%%
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

for i = 1:1200  %x1 sec (Ts)
    tic; 
    t1 = T1C();
    t2 = T2C();
    y = [t1-Tamb; t2-Tamb];
    y_hat(:,i) = Cd*x(:,i);

    u(:,i) = Kr*r(:,i) - K*x(:,i);
    
    if u(1,i) <= 0
        u(1,i) = 0;
    end
    if u(2,i) <= 0
        u(2,i) = 0;
    end
    
    ht1 = u(1,i);
    ht2 = u(2,i);
    h1(ht1);
    h2(ht2);
  
    x(:,i+1) = Ad*x(:,i) + Bd*u(:,i) + L*(y - y_hat(:,i));
    
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
    
    t = toc;
    pause(max(0.01,1.0-t))
end

h1(0);
h2(0);

%%
figure
subplot(2,1,1)
plot(linspace(1,length(r),length(r)),r(1,:)+Tamb,'k')
hold on
plot(linspace(1,length(r),length(r)),r(2,:)+Tamb,'k')
plot(linspace(1,length(t1s),length(t1s)),t1s,'Color','y');
plot(linspace(1,length(x),length(x)),x(3,:)+Tamb,'LineWidth',1,'Color','b');
plot(linspace(1,length(t2s),length(t2s)),t2s,'Color','g');
plot(linspace(1,length(x),length(x)),x(4,:)+Tamb,'LineWidth',1,'Color','r');
legend('Ref','Ref','T1','T1 observed','T2','T2 observed');
ylabel('Temperature (deg)')
xlabel('Time (s)')
xlim([0 length(u)])
ylim([15 45])

subplot(2,1,2)

plot(linspace(1,length(u),length(u)),u(1,:),'LineWidth',1);
hold on
plot(linspace(1,length(u),length(u)),u(2,:),'LineWidth',1);
legend('U1','U2');
ylabel('Input (%)')
xlabel('Time (s)')