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

Q = 100;
R = eye(2);
Q_terminal = 100;

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

r_input = repmat(20,1,dim.N+1); %20 deg


%% Running controller

clf
hold on

x = x0;
implementedU = [];
implementedUtotal = [];
X0 = [];

for i = 1:900 
  
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

figure(2)
stairs(implementedUtotal(1,:),'p') %implementedUtotal
hold on
stairs(implementedUtotal(2,:),'g') %implementedUtotal


%% Check 

x_d = x0;
for i = 1:900
    x_d(:,i+1) = sys.A*x_d(:,i)+sys.B*implementedUtotal(:,i);
end

figure(3)
plot(linspace(1,length(x_d),length(x_d)),x_d)

figure(4)
plot(linspace(1,length(x_d),length(x_d)),x_d(4,:))



