clearvars
close all

load('MIMO_para2')
load('data3.mat')

%% Linearized discretized system

%x1 = temperature heater 1 (Th1) (Celsius)
%x2 = temperature heater 2 (Th2) (Celsius)
%x3 = temperature sensor 1 (Tc1) (Celsius)
%x4 = temperature sensor 2 (Tc2) (Celsius)

%u1 = power input heater 1 (Q1) (0 to 1 Watt)
%u2 = power input heater 2 (Q2) (0 to 0.75 Watt)

sys.A = lin_discrete.A;
sys.B = lin_discrete.B;
sys.C = lin_discrete.C;
sys.D = lin_discrete.D;

[dim.rowsB,dim.colB] = size(sys.B);
[dim.rowsC,dim.colC] = size(sys.C);
                  
Ts = 1;                    %timestep equal to 1 sec
%K = 273.15;
x0=[0;0;0;0];              %initial state values 21 deg C
Tamb =21;

dim.nx=length(sys.A);      %state dimension
dim.ny=dim.rowsC;          %output dimension
dim.nu=dim.colB;           %input dimension
dim.N=300;                 %400 seconds (i.e. 6,6 min)

%% Formulating constraints
% we have upper and lower bound for u 
% and upper and lower bound for x

constr.lowu1 = 0;
constr.upu1 = 100;
constr.lowu2 = 0;
constr.upu2 = 100;

constr.lowx = -21; % subtracted Tamb
constr.upx = 80; 

%% Formulating objective (xQx + uRu) with constraints

u = sdpvar(repmat(dim.nu,1,dim.N),repmat(1,1,dim.N));
x = sdpvar(repmat(dim.nx,1,dim.N+1),repmat(1,1,dim.N+1));
r = sdpvar(repmat(1,1,dim.N+1),repmat(1,1,dim.N+1));


% Q = [100 0; 0 100];
% R = eye(2);
% Q_terminal = [100 0; 0 100];
% 
% Q = [200 0; 0 400];
% R = eye(2);
% Q_terminal = [200 0; 0 400];

Q = [200 0; 0 400];
R = eye(2)*(1E-1)*5;
Q_terminal = [200 0; 0 400];


constraints = [];
objective = 0;

for k = 1:dim.N
    objective = objective + (sys.C*x{k}-r{k})'*Q*(sys.C*x{k}-r{k}) + u{k}'*R*u{k};
    if k == dim.N 
         objective = objective + (sys.C*x{k+1}-r{k})'*Q_terminal*(sys.C*x{k+1}-r{k});
    end
    constraints = [constraints, x{k+1} == sys.A*x{k} + sys.B*u{k}];
    constraints = [constraints, constr.lowu1 <= u{k}(1) <= constr.upu1];
    constraints = [constraints, constr.lowu2 <= u{k}(2) <= constr.upu2];  
    constraints = [constraints, constr.lowx <= x{k} <= constr.upx];
end
 
%% Making controller

options = sdpsettings('solver','quadprog');

parameters_in = {x{1},[r{:}]};
controller = optimizer(constraints, objective, options, parameters_in,{[u{:}],[x{:}]});

% % constant reference of 40 deg
% r_input = repmat(20,1,dim.N+1); 
% 
% % switching straight reference
% r_input1 = repmat(20,1,dim.N); %40 deg
% r_input2 = repmat(40,1,2*dim.N); %60 deg
% ref = [r_input1 r_input2];
%
% % sinus wave reference

sine1 = dsp.SineWave(2,10);
sine1.SamplesPerFrame = 1000;
y = sine1();
r_input = 3*y + 20;
ref = r_input';


%% Running controller on TClab

% tclab;
% clf
% hold on
% 
% x0 = [T1C()-Tamb; T2C()-Tamb; T1C()-Tamb; T2C()-Tamb];
% x = x0;
% implementedU = [];
% implementedUtotal = [];
% X0 = [];
% 
% Rlqr = eye(2)*1E3;
% %Qlqr = eye(4);
% Qlqr = [80 0 0 0; 0 80 0 0; 0 0 40 0; 0 0 0 40];
% 
% %[Kest,L,P2] = kalman(lin_discrete,[],[],[]);
% [K,S,CLP] = dlqr(sys.A',sys.C',Qlqr,Rlqr,[]);
% 
% h1s = [];
% h2s = [];
% t1s = [];
% t2s = [];
% OUTPUT = [];
% tic
% for i = 1:200
%    toc
%    tic 
%   [solution,~] = controller{x,r_input};  
%   U = solution{1};
%   X = solution{2};
%   
%     ht1 = U(1,1);
%     ht2 = U(2,1);
%     h1(ht1);
%     h2(ht2);
%     
%     t1 = T1C();
%     t2 = T2C();
%   
%     %x = sys.A*x + sys.B*U(:,1);
%     y = [t1-Tamb; t2-Tamb];
%     y_hat = sys.C*x;
%     %y = y_hat(:,1);
%     OUTPUT = [OUTPUT x];
%     x = sys.A*x + sys.B*U(:,1)+ K'*(y - y_hat);
%     
%   stairs(i:i+length(U)-1,U(1,:)','r')
%   hold on
%   stairs(i:i+length(U)-1,U(2,:)','b')
%   
%   implementedUtotal = [implementedUtotal U(:,1)];
%   X0 = [X0 X(:,1)];
%   
%     h1s = [h1s,ht1];
%     h2s = [h2s,ht2];
%     t1s = [t1s,t1];
%     t2s = [t2s,t2];
%     
%     n = length(t1s);
%     time = linspace(0,(n+1),n);
%     
%     clf
%     subplot(2,1,1)
%     plot(time,t1s,'b.','MarkerSize',10);
%     hold on
%     plot(time,t2s,'r.','MarkerSize',10);
%     ylabel('Temperature (degC)')
%     legend('Temperature 1','Temperature 2','Location','NorthWest')
%     
%     subplot(2,1,2)
%     plot(time,h1s,'b-','LineWidth',2);
%     hold on
%     plot(time,h2s,'r--','LineWidth',2);
%     ylabel('Heater (0-5.5 V)')
%     xlabel('Time (sec)')
%     legend('Heater 1','Heater 2','Location','NorthWest')
%     drawnow;
%     pause(Ts-0.65)
% end
% toc
% h1(0);
% h2(0);
% display('Heaters off')
% 
% figure(2)
% stairs(implementedUtotal(1,:),'p') %implementedUtotal
% hold on
% stairs(implementedUtotal(2,:),'g') %implementedUtotal
% 
% T = [t1s;t2s] - ones(2,length(t1s))*Tamb;
% 
% figure(3)
% % subplot(2,1,1)
% plot(linspace(1,length(T),length(T)),T(1,:))
% hold on 
% plot(linspace(1,length(T),length(T)),OUTPUT(3,:))
% plot(linspace(1,length(T),length(T)),T(2,:))
% plot(linspace(1,length(T),length(T)),OUTPUT(4,:))
% legend('measured T1','observer T1','measured T2','observer T2')





%% Running controller on model

clf
hold on

x = x0;
implementedU = [];
implementedUtotal = [];
X0 = [];

for i = 1:500 
  r_input = ref(:,i:(i+dim.N)); 
  [solution,~] = controller{x,r_input};  
  U = solution{1};
  X = solution{2};
  stairs(i:i+length(U)-1,U(1,:)','r')
  hold on
  stairs(i:i+length(U)-1,U(2,:)','b')
  x = sys.A*x + sys.B*U(:,1);
  pause(0.05)
  stairs(i:i+length(U)-1,U(1,:)','k')
  implementedUtotal = [implementedUtotal U(:,1)];
  X0 = [X0 X(:,1)];
end


%%
figure(2)
stairs(implementedUtotal(1,:),'color','r','lineWidth',1) %implementedUtotal
hold on
stairs(implementedUtotal(2,:),'color','b','lineWidth',1) %implementedUtotal
legend('Input heater 1','Input heater 2')
xlabel('Time [sec]')
ylabel('Power Input [%]')
set(gca,'FontSize',14)


%% Check 

x_d = x0;
for i = 1:500
    x_d(:,i+1) = sys.A*x_d(:,i)+sys.B*implementedUtotal(:,i);
end

figure(3)
plot(linspace(1,length(x_d),length(x_d)),x_d+20,'lineWidth',1)
legend('Temperature heater 1','Temperature heater 2','Temperature sensor 1','Temperature sensor 2','Location','northwest')
xlabel('Time [sec]')
ylabel('Temperature [Celsius]')
set(gca,'FontSize',14)

figure(4)
plot(linspace(1,length(x_d),length(x_d)),x_d(3,:)+20,'color','r','lineWidth',1)
hold on
plot(linspace(1,length(x_d),length(x_d)),x_d(4,:)+20,'color','b','lineWidth',1)
legend('Temperature sensor 1','Temperature sensor 2','Location','northwest')
xlabel('Time [sec]')
ylabel('Temperature [Celsius]')
set(gca,'FontSize',14)




