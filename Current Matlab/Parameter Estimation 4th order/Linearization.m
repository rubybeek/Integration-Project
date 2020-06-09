load('coefficients.mat')
load('parameters2.mat')
load('data4.mat')

%Tamb = min(data(1,2),data(1,3));
Tamb = 21;
data(:,2:3) = data(:,2:3) - Tamb;

Am = zeros(4,4);
Bm = zeros(4,2);
Cm = zeros(2,4);
Dm = zeros(2,2);

%T0 = 19;
T0 = 273.15+19; %linearization temperature (19+21=40 degree)
c1 = U*A;
c2 = 4*eps*sigma*A*T0^3;
c3 = Us*As;
c4 = 4*eps*sigma*As*T0^3;
c5 = m*Cp;
c6 = 1/tau;

Am(1,1) = -(c1+c2+c3+c4)/c5;
Am(1,2) = (c3+c4)/c5;

Am(2,1) = (c3+c4)/c5;
Am(2,2) = -(c1+c2+c3+c4)/c5;

Am(3,1) = c6;
Am(3,3) = -c6;

Am(4,2) = c6;
Am(4,4) = -c6;

Bm(1,1) = alpha1/c5;
Bm(2,2) = alpha2/c5;

Cm(1,3) = 1;
Cm(2,4) = 1;

Dm = [];

Ts = 1;
lin_statespace = ss(Am,Bm,Cm,Dm);

x0 = [Tamb ; Tamb ; Tamb ; Tamb ];

[y,t,x] = lsim(lin_statespace,data(:,4:5),data(:,1));

figure(3)

subplot(2,1,1)
plot(data(:,1),data(:,2))
hold on
plot(t,(y(:,1)))
legend('measured data','linearized model')
ylabel('T1 [C - 21C]')
xlabel('time (t)')

subplot(2,1,2)
plot(data(:,1),data(:,3))
hold on
plot(t,(y(:,2)))
legend('measured data','linearized model')
ylabel('T2 [C - 21C]')
xlabel('time (t)')

% Conclusion: linearization is only accurate when operating in temperatures
% close to the temperature around which is linearized. Thus for the ss
% description to fit the real system, a dataset with a not so much varying
% temp needs to be used... 

lin_discrete = c2d(lin_statespace,Ts);

save('MIMO_para2','lin_statespace','lin_discrete')

%% VAF 

dataset = iddata(data(:,2:3),data(:,4:5),1);
dataset.InputName  = {'Input Sensor 1';'Input Sensor 2'};
dataset.OutputName = {'Output Sensor 1';'Output Sensor 2'};

figure(4)
compare(dataset,lin_discrete);




