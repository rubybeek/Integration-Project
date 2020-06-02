clearvars
close all

%u1 = heater [W] %u2 = outside temp [C]
%u3 = people [W] %u4 = solar radiation [W]

%x1 = floor [C] %x2 = internal wall [C]
%x3 = ext. wall [C] %x4 = room temp [C]
load('MIMO_para2')

%%
days = 2;                   %data is from 01/01/2020 until 16/03/2020
Ts = 600;                   % 10 min, Ts always in seconds
K = 273.15;
x0=[20;15;10;20];           %initial state values
%x0=[20;20;20;20];

%[Temp,Solar,Humans] = dataload(Ts,days); %load data

%[sysc, sysd] = dynamics_build(Ts,x0);   %compute continuous and discrete state space system
A = lin_discrete.A;
B =lin_discrete.B;
C =lin_discrete.C;
D =lin_discrete.D;

dim.nx=length(A);          %state dimension
dim.ny=1;                       %output dimension
dim.nu=length(B);          %input dimension
dim.N=24;                      %horizon %2hours

%Definition of quadratic cost function
% Q=eye(dim.ny);                  %weight on output
% R=eye(dim.nu);                  %weight on input

%% Check model
% x_d = zeros([4,100]);
% x_d(:,1) = x0;
% u = [0;20;0;0]; 
% 
% for i = 1:100
%     x_d(:,i+1) = sysd.A*x_d(:,i)+sysd.B*u;
% end

%% Control with prediction matrix (obj = 0.5*u'*H*u+h'*u)
% B = sysd.B;

% % Prediction model and cost function
% ref = 20*ones(dim.N,1);
% 
% [P,S]=predmodgen(sysd,B,dim);            %Generation of prediction model 
% [H,h,const]=costgen(P,S,Q,R,dim,x0,ref);  %Writing cost function in quadraticform
% %quadcost=quadprobgen(P,S,weight,dim);
% 
% % optimize
% 
% u = sdpvar(dim.N*4,1);                % define optimization variable 5x1
% x = sdpvar(dim
% %u = sdpvar(repmat(dim.nu,1,dim.N),repmat(1,1,dim.N));
% 
% [A,A2,b,b2,b3]=constraintgen(Humans,Solar,Temp);
% 
% Constraints = [A*u==b,A2*x];       %define constraints
% 
% Objective = 0.5*u'*H*u+h'*u;  %define cost function
% 
% % Set some options for YALMIP and solver
% % options = sdpsettings('verbose',1,'solver','quadprog','quadprog.maxiter',100);
% 
% optimize(Constraints,Objective);  %solve the problem
% uopt=value(u);                   %assign the solution to uopt1

%% Control with normal objective (xQx + uRu)

%[b,Arep]=constraintgen(Humans,Solar,Temp,days,Ts);

u = sdpvar(repmat(dim.nu,1,dim.N),repmat(1,1,dim.N));
x = sdpvar(repmat(dim.nx,1,dim.N+1),repmat(1,1,dim.N+1));
%r = sdpvar(repmat(1,1,dim.N+1),repmat(1,1,dim.N+1));
r = sdpvar(2,1);
%b_sdpvr = sdpvar(3*dim.N,1);
%r = 20;  % make decision var if optimizer command is used

%%

%Q = 10000;
%Q = sdpvar(1);
Q = sdpvar(repmat(1,1,dim.N),repmat(1,1,dim.N));
xlowlim = sdpvar(repmat(1,1,dim.N),repmat(1,1,dim.N));
R = [ 0.0001 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Q_terminal = 100;
ulowlim = 100;
uuplim = 0;
%xlowlim = 15;
xuplim = 22;

constraints = [];
objective = 0;

for k = 1:dim.N
    objective = objective + (C*x{k}-r{k})'*Q{k}*(C*x{k}-r{k}) + u{k}'*R*u{k};
    if k == dim.N 
         objective = objective + (x{k+1}-r{k})'*Q_terminal*(C*x{k+1}-r{k});
    end
    constraints = [constraints, x{k+1} == A*x{k} + B*u{k}];
    %constraints = [constraints, Arep*u{k} == b_sdpvr(k*3-2:k*3,:)];
    %constraints = [constraints, Arep*u{k} == b(k*3-2:k*3,:)];
    constraints = [constraints, ulowlim <= u{k}(1) <= uuplim];
    %constraints = [constraints, xlowlim{k} <= x{k}(4) <= xuplim];
end

%% Using optimize
% constraints = [constraints, x{1} == x0]; 
% options = sdpsettings('solver','quadprog');

% optimize(constraints,objective,options);

% uopt = value(u);
% xopt = value(x);

%% Using optimizer
options = sdpsettings('solver','quadprog');

% parameters_in = {x{1},b_sdpvr};
% parameters_in = {x{1},[r{:}]};
% parameters_in = {x{1}};
parameters_in = {x{1},[r{:}]};
controller = optimizer(constraints, objective,options,parameters_in,{[u{:}],[x{:}]});

r_input = repmat(20,1,dim.N+1);
% b_input = b(1:3*dim.N,:);
% U = controller{x0,b_input};
% U = controller{x0,r_input,b_input};
% U = controller{x0,r_input};
% U = controller{x0};

%%

clf
hold on

x = x0;
implementedU = [];
implementedUtotal = [];
X0 = [];
Q = [];
xlowlim = [];

for i = 1:(24*6*2)
     
  b_input = b(i*3-2:3*dim.N + i*3 - 3,:);
  
  for j = 1:dim.N
    if b_input(j*3-1) == 0;
      Q(j) = 0;
      xlowlim(j) = 15;
    else
      Q(j) = 10000;
      xlowlim(j) = 19;
    end
  end
  
  [solution,~] = controller{x,r_input,b_input,Q,xlowlim};  
  U = solution{1};
  X = solution{2};
  stairs(i:i+length(U)-1,U(1,:)','r')
  x = sysd.A*x + sysd.B*U(:,1);
  pause(0.05)
  stairs(i:i+length(U)-1,U(1,:)','k')
  implementedU = [implementedU;U(1,2)];
  implementedUtotal = [implementedUtotal U(:,1)];
  X0 = [X0 X(:,1)];
end

figure(2)
stairs(implementedU,'b')

%% Check 

x_d = x0;
for i = 1:(24*6*2)
    x_d(:,i+1) = sysd.A*x_d(:,i)+sysd.B*implementedUtotal(:,i);
end

figure(3)
plot(linspace(1,length(x_d),length(x_d)),x_d)

figure(4)
plot(linspace(1,length(x_d),length(x_d)),x_d(4,:))
