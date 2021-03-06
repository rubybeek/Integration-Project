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
% 
% Ac = [Ac zeros(4,1) ; 0 0 0 0 1];
% Bc = [Bc zeros(4,1); 0 0 -1];
% Cc = [Cc zeros(2,1)];
% Dc = 0;
% lin_statespace = ss(Ac,Bc,Cc,Dc); 

% Ac = [Ac zeros(4,2) ; Cc eye(2)];
% Bc = [Bc; 0 0 ; 0 0]; % zeros(4,1) ;0 0 -1; 0 0 -1];
% Cc = [Cc zeros(2,2)];
% Dc = 0;
% lin_statespace = ss(Ac,Bc,Cc,Dc); 
%%

Poles = [pole(lin_statespace)]'; %[-0.0313   -0.0313   -0.0044   -0.0134]
Poles_disc = [pole(lin_discrete)]';
%P = [-0.0313   -0.0313   -0.0044   -0.0134]; 
% p3 for faster T2 response, p4 for faster T1 response
%P = [-0.035   -0.035   -0.0120   -0.0150];

P = [-0.034   -0.034   -0.0120   -0.0200 ]; %THIS IS THE ONE
K = place(Ac,Bc,P);

newsys = ss(Ac-Bc*K,Bc,Cc,Dc);
newsys_disc = c2d(newsys,1,'zoh'); 
Poles_dis_new = pole(newsys_disc);
MIMO = tf(newsys(1:2,1:2));
MIMO = tf(newsys);

figure(1)
step(MIMO)

% figure(2)
% step(lin_statespace)

Kdc = dcgain(MIMO);
Kr = inv(Kdc(1:2,1:2));

%% Observer Testing
%what we had: eye(2)*100,eye(2)*10E-2,eye(2)
Qkal = [40 0; 0 40];
Rkal =eye(2)*(1E-2)*5;
Nkal =eye(2);

[KEST,L,Pkal] = kalman(lin_discrete,Qkal,Rkal,Nkal);
%[X,Kobs,L] = idare(Ad,Bd,Qlqr,Rlqr,[],[]); 

load('data3.mat')
uobs = data(:,3:4)';
x_data = data(:,1:2)';
xobs(:,1) = [21.8475; 20.8700; 21.8475; 20.8700];

for i = 1:length(data)-1
    yobs = x_data(:,i);
    y_hatobs = Cd*xobs(:,i);
    xobs(:,i+1) = Ad*xobs(:,i) + Bd*uobs(:,i)+ L*(yobs - y_hatobs);
end

figure
plot(linspace(1,length(data),length(data)),x_data(1,:))
hold on
plot(linspace(1,length(data),length(data)),xobs(3,:))
plot(linspace(1,length(data),length(data)),x_data(2,:))
plot(linspace(1,length(data),length(data)),xobs(4,:))
legend('measured T1','Observed T1','measured T2','Observed T2')

%% Simulation
% Kx = K(1:2,1:4);
% Ki = [0.1 0.1; 0.1 0.1];
% x(:,1) = [0; 0; 0; 0];
% %z(:,1) = [0;0];
% 
% for i = 1:3000
%     if i <= 1000
%         r(:,i) = [19;19]; %40 degree
%     elseif (i > 1000) && (i <= 2000);
%         r(:,i) = [14;14]; %35 degree
%     else 
%         r(:,i) = [24;24]; %45 degree
%     end
%     %u(:,i) = Kr*r(:,i) - Kx*x(:,i) - Ki*z(:,1);
%     z(:,i) = Cd*x(:,i) - r(:,i);
%     u(:,i) = Kr*r(:,i) - K*x(:,i) - Ki*z(:,i);
%     
%     if u(1,i) <= 0
%         u(1,i) = 0;
%     end
%     if u(2,i) <= 0
%         u(2,i) = 0;
%     end
% 
%     x(:,i+1) = Ad*x(:,i) + Bd*u(:,i);% + L*(y - y_hat(:,i));
% end
% 
% figure()
% plot(linspace(1,length(x),length(x)),x(3,:))
% hold on
% plot(linspace(1,length(x),length(x)),x(4,:))
% plot(linspace(1,length(r),length(r)),r(1,:))

%% TCLAB
Tamb = 21;
r = [19;19];
tclab;
x(:,1) = [T1C()-Tamb; T2C()-Tamb; T1C()-Tamb; T2C()-Tamb];
%x(:,1) = [0; 0; 0; 0];

h1s = [];
h2s = [];
t1s = [];
t2s = [];

ht1 = 0;
ht2 = 0;
h1(ht1);
h2(ht2);

switching ref, different per sensor
r_input1(1,:) = repmat(19,1,600); %40 deg
r_input1(2,:) = [repmat(10,1,500),repmat(14,1,300)]; %40 deg
r_input2(1,:) = [repmat(14,1,500),repmat(22,1,300)]; %35 deg
r_input2(2,:) = repmat(14,1,600); %35 deg
r_input3(1,:) = repmat(22,1,600); %45 deg
r_input3(2,:) = repmat(19,1,600); %45 deg
ref = [r_input1 r_input2 r_input3];

for i = 1:1000  %x1 sec (Ts)
    tic;
%     if i <= 1000
%         r(:,i) = [19;19]; %40 degree
%     elseif (i > 1000) && (i <= 2000);
%         r(:,i) = [14;14]; %35 degree
%     else 
%         r(:,i) = [24;24]; %45 degree
%     end
    y_hat(:,i) = Cd*x(:,i);
    z(:,i) = y_hat(:,i) - r(:,i);
    u(:,i) = Kr*r(:,i) - K*x(:,i) - z(:,i);
    
    if u(1,i) <= 0
        u(1,i) = 0;
    end
    if u(2,i) <= 0
        u(2,i) = 0;
    end
    if u(1,i) >= 100
        u(1,i) = 100;
    end
    if u(2,i) >= 100
        u(2,i) = 100;
    end
    
    
    ht1 = u(1,i);
    ht2 = u(2,i);
    h1(ht1);
    h2(ht2);
    t1 = T1C();
    t2 = T2C();
    y = [t1-Tamb; t2-Tamb];
    %y_hat(:,i) = Cd*x(:,i);
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

%
figure
subplot(2,1,1)
%plot(linspace(1,length(x),length(x)),[ones(1,500)*19, ones(1,500)*14, ones(1,501)*24],'k')
hold on
plot(linspace(1,length(t1s),length(t1s)),t1s-Tamb,'color','y');
plot(linspace(1,length(x),length(x)),x(3,:),'LineWidth',1,'Color','b');
plot(linspace(1,length(t2s),length(t2s)),t2s-Tamb,'Color','g');
plot(linspace(1,length(x),length(x)),x(4,:),'LineWidth',1,'Color','r');
legend('Ref','T1','T1 observed','T2','T2 observed');
% plot(linspace(1,length(x),length(x)),x(1,:));
% plot(linspace(1,length(x),length(x)),x(2,:));
ylabel('Temperature - 21deg (deg)')
xlabel('Time (s)')
xlim([0 length(u)])


subplot(2,1,2)
plot(linspace(1,length(u),length(u)),u(1,:),'LineWidth',1);
hold on
plot(linspace(1,length(u),length(u)),u(2,:),'LineWidth',1);
legend('U1','U2');
ylabel('Input (%)')
xlabel('Time (s)')

%% some analysis 
% tf_parest = tf(lin_statespace);
% Ktf = tf(K);
% L = tf_parest*Ktf(:,3:4); 
% S1 = inv(eye(2)+tf_parest);
% %S = inv(eye(2)+tf_parest*Ktf(:,3:4)); %this is bullshit?
% S = minreal(inv(eye(size(L))+L)); %this is bullshit?
% 
% figure
% bode(S1(1,1))
% hold on 
% bode(S(1,1))
% legend('sensitivity ol','sensitivity cl')

