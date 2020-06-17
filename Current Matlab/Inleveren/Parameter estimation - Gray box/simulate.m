% Adjusted code based on code from:
% http://apmonitor.com/pdc/index.php/Main/ArduinoTemperatureControl

function T = simulate(p)

global t T1meas T2meas Q1 Q2

T = zeros(length(t),2);
T(1,1) = T1meas(1);
T(1,2) = T2meas(1);
T0 = [T(1,1),T(1,2),T(1,1),T(1,2)];
for i = 1:length(t)-1
    ts = [t(i),t(i+1)];
    sol = ode15s(@(t,x)heat(t,x,Q1(i),Q2(i),p),ts,T0);
    T0 = [sol.y(1,end),sol.y(2,end),sol.y(3,end),sol.y(4,end)];
    T(i+1,1) = T0(3);
    T(i+1,2) = T0(4);
end
end