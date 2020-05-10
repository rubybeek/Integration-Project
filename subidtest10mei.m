clearvars

load('data.1.mat')
dataset = iddata(data(:,1:2),data(:,3:4),1);
dataset.InputName  = {'Input Sensor 1';'Input Sensor 2'};
dataset.OutputName = {'Output Sensor 1';'Output Sensor 2'};





%%

% Subspace identification

y1 = data(:,1);
y2 = data(:,2);
y = [y1' ; y2'];
 
u1 = data(:,3);
u2 = data(:,4);

u = [u1' ; u2'];
 
p = 2;

R = 3 * p;

 
[A,B,C,D,n] = moespar(u,y,R)


tf_system1 = ss(A,B,C,D);



Ts = 1;
discrete_system1 = c2d(tf_system1,1);

x = [];
x(:,1) = y(:,1);

for k = 1:399

    x(:,k+1) = discrete_system1.A * x(:,k) + discrete_system1.B * u(:,1);
   
end

t = linspace(1,400,400);

%%

figure
plot(t,x(1,:))
hold on
plot(t,x(2,:))

%%
MIMO = ssest(dataset);

system = c2d(MIMO,1);

x2 = [];
x2(:,1) = y(:,1);


for k = 1:399

    x2(:,k+1) = system.A * x2(:,k) + system.B * u(:,1);
   
end

figure
plot(t,x2(1,:))
hold on
plot(t,x2(2,:))


