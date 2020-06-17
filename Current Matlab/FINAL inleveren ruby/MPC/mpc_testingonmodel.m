% ------------------------------------------------%
% MPC Controller - testing on model
% ------------------------------------------------%
% Note1: select the derisered reference trajectory
% in line 92 t/m 124
% Note2: add Yalmip to path
% ------------------------------------------------%

clearvars
close all

load('Lineardiscretemodel.mat')
load('Dataset.mat')
load('Referencesinewave.mat')

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

%% Defining the dimensions
                  
Ts = 1;                    %timestep equal to 1 sec

x0=[0;0;0;0];              %initial state values 21 deg C
Tamb =21;                  %Ambient temperature

dim.nx=length(sys.A);      %state dimension
dim.ny=dim.rowsC;          %output dimension
dim.nu=dim.colB;           %input dimension
dim.N=300;                 %horizon 300 seconds (i.e. 5 min)

%% Formulating constraints

% Upper and lower bounds on heaters (in percentage)
constr.lowu1 = 0;   
constr.upu1 = 100;
constr.lowu2 = 0;
constr.upu2 = 100;

% Upper and lower bound on temperatures (it is not desired that the system
% exceeds the boundaries of 0 < T < 100 degrees C

constr.lowx = -21; % subtracted Tamb
constr.upx = 80;   % subtracted Tamb

%% Formulating objective with constraints

u = sdpvar(repmat(dim.nu,1,dim.N),repmat(1,1,dim.N));
x = sdpvar(repmat(dim.nx,1,dim.N+1),repmat(1,1,dim.N+1));
r = sdpvar(repmat(2,1,dim.N+1),repmat(1,1,dim.N+1));

% Tuned weight matrices
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
 
%% Making controller using Yalmip

options = sdpsettings('solver','quadprog');

parameters_in = {x{1},[r{:}]};
controller = optimizer(constraints, objective, options, parameters_in,{[u{:}],[x{:}]});

%% Reference creation
% De-comment which reference is desired to observe

% % constant reference of 40 deg
% ref = repmat(19,2,2500); 
 
% % switching straight reference
% r_input1 = repmat(19,2,500); %40 deg
% r_input2 = repmat(14,2,500); %35 deg
% r_input3 = repmat(24,2,500+2000); %45 deg
% ref = [r_input1 r_input2 r_input3];


% switching ref, different per sensor
r_input1(1,:) = repmat(19,1,400); %40 deg
r_input1(2,:) = [repmat(10,1,300),repmat(14,1,100)]; %40 deg
r_input2(1,:) = [repmat(14,1,300),repmat(22,1,100)]; %35 deg
r_input2(2,:) = repmat(14,1,400); %35 deg
r_input3(1,:) = repmat(22,1,400+2000); %45 deg
r_input3(2,:) = repmat(19,1,400+2000); %45 deg
ref = [r_input1 r_input2 r_input3];

% % sinus wave reference
% ref = ref'-Tamb;
% ref = [ref;ref];

% Plotting the reference trajectories
figure(1)
plot(linspace(1,length(ref),length(ref)),ref(:,1:end)+Tamb,'LineWidth',1)
hold on
legend('Ref')
xlabel('Time (s)')
ylabel('Temperature (Deg)')

%% Running controller on model

clf
hold on

x = x0;
implementedU = [];
implementedUtotal = [];
X0 = [];

for i = 1:1500 
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

%% Plotting input

figure(2)
stairs(implementedUtotal(1,:),'color','r','lineWidth',1) 
hold on
stairs(implementedUtotal(2,:),'color','b','lineWidth',1) 
legend('Input heater 1','Input heater 2')
xlabel('Time [sec]')
ylabel('Power Input [%]')
set(gca,'FontSize',14)
xlim([1 1500])

%% Plotting all outputs

x_d = x0;
for i = 1:1500
    x_d(:,i+1) = sys.A*x_d(:,i)+sys.B*implementedUtotal(:,i);
end

figure(3)
plot(linspace(1,length(x_d),length(x_d)),x_d+Tamb,'lineWidth',1)
legend('Temperature heater 1','Temperature heater 2','Temperature sensor 1','Temperature sensor 2','Location','northwest')
xlabel('Time [sec]')
ylabel('Temperature [Celsius]')
set(gca,'FontSize',14)
xlim([1 1500])

%% Plotting the sensor outputs and the reference
figure(4)
plot(linspace(1,length(x_d),length(x_d)),x_d(3,:)+Tamb,'color','r','lineWidth',1)
xlim([1 1500])
hold on
plot(linspace(1,length(x_d),length(x_d)),x_d(4,:)+Tamb,'color','b','lineWidth',1)
plot(linspace(1,length(ref),length(ref)),ref(:,1:end)+Tamb,'LineWidth',1,'color','k')
legend('Temperature sensor 1','Temperature sensor 2','Reference','Location','southeast')
xlabel('Time [sec]')
ylabel('Temperature [Celsius]')
set(gca,'FontSize',14)






