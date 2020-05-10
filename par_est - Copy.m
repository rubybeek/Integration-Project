function [dy2dt2_1,dy2dt2_2,dydt1, dydt2] = par_est(data, x, t, Kp, zetap, taup, thetap)

Q1_0 = data(1,3);
Q2_0 = data(1,4);
T1_0 = data(1,1);
T2_0 = data(1,2);

Q1 = data(:,3);
Q2 = data(:,4);
T1 = data(:,1);
T2 = data(:,2);

y1 = x(1);
y2 = x(2);
dydt1 = x(3);
dydt2 = x(4);
dy2dt2_1 = (-2*zetap*taup*dydt1 - y1 + Kp * Q1*(t - taup))/(taup^2);
dy2dt2_2 = (-2*zetap*taup*dydt2 - y2 + Kp * Q2*(t - taup))/(taup^2);

end
