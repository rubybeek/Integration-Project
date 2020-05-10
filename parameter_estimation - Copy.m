
% % initial guessess
% x0 = zeros(4);
% x0(0) = 0.8; % Kp
% x0(1) = 0.2; % Kd
% x0(2) = 150.0; % taup
% x0(3) = 10.0; % thetap

load data.1.mat

% Initial guesses
Kp = 2;
taup = 1;
thetap = 1;
zetap = 1;

[dy2dt2_1,dy2dt2_2,dydt1, dydt2] = par_est(data, x, t, Kp, zetap, taup, thetap);





% define first-order plus dead-time approximation 

function [dTdt,dT2dt] = fopdt(T,t,Q1,Q2,Kp,Kd.taup,thetap)
    
        if  (t-thetap) <= 0
            Qm1 = Q1(0.0);
            Qm2 = Q2(0.0);
        else
            Qm1 = Q1(t-thetap);
            Qm2 = Q2(t-thetap);
        end
    % calculate derivative
    dT1dt = (-(T(1)-T1_0) + Kp*(Qm1-Q1_0) + Kd*(T(2)-T(1)))/taup;
    dT2dt = (-(T(2)-T2_0) + (Kp/2.0)*(Qm2-Q2_0) + Kd*(T(1)-T(2)))/taup;

end

        

% simulate FOPDT model
def sim_model(x):
    # input arguments
    Kp,Kd,taup,thetap = x
    # storage for model values
    T1p = np.ones(ns) * T1_0
    T2p = np.ones(ns) * T2_0
    # loop through time steps    
    for i in range(0,ns-1):
        ts = [t[i],t[i+1]]
        T = odeint(fopdt,[T1p[i],T2p[i]],ts,args=(Qf1,Qf2,Kp,Kd,taup,thetap))
        T1p[i+1] = T[-1,0]
        T2p[i+1] = T[-1,1]
    return T1p,T2p

%1 define objective
def objective(x):
    # simulate model
    T1p,T2p = sim_model(x)
    # return objective
    return sum(np.abs(T1p-T1)+np.abs(T2p-T2))  

# show initial objective
print('Initial SSE Objective: ' + str(objective(x0)))
print('Optimizing Values...')

# optimize without parameter constraints
#solution = minimize(objective,x0)

# optimize with bounds on variables
bnds = ((0.4, 1.5), (0.1, 0.5), (50.0, 200.0), (0.0, 30.0))
solution = minimize(objective,x0,bounds=bnds,method='SLSQP')

# show final objective
x = solution.x
iae = objective(x)
Kp,Kd,taup,thetap = x
print('Final SSE Objective: ' + str(objective(x)))
print('Kp: ' + str(Kp))
print('Kd: ' + str(Kd))
print('taup: ' + str(taup))
print('thetap: ' + str(thetap))
# save fopdt.txt file
fid = open('fopdt.txt','w')
fid.write(str(Kp)+'\n')
fid.write(str(Kd)+'\n')
fid.write(str(taup)+'\n')
fid.write(str(thetap)+'\n')
fid.write(str(T1_0)+'\n')
fid.write(str(T2_0)+'\n')
fid.close()

# calculate model with updated parameters
T1p,T2p = sim_model(x)

plt.figure(1,figsize=(15,7))
plt.subplot(2,1,1)
plt.plot(t,T1,'r.',linewidth=2,label='Temperature 1 (meas)')
plt.plot(t,T2,'b.',linewidth=2,label='Temperature 2 (meas)')
plt.plot(t,T1p,'r--',linewidth=2,label='Temperature 1 (pred)')
plt.plot(t,T2p,'b--',linewidth=2,label='Temperature 2 (pred)')
plt.ylabel(r'T $(^oC)$')
plt.text(200,20,'Integral Abs Error: ' + str(np.round(iae,2)))
plt.text(400,35,r'$K_p$: ' + str(np.round(Kp,2)))  
plt.text(400,30,r'$K_d$: ' + str(np.round(Kd,2)))  
plt.text(400,25,r'$\tau_p$: ' + str(np.round(taup,1)) + ' sec')  
plt.text(400,20,r'$\theta_p$: ' + str(np.round(thetap,1)) + ' sec')  
plt.legend(loc=2)
plt.subplot(2,1,2)
plt.plot(t,Q1,'b--',linewidth=2,label=r'Heater 1 ($Q_1$)')
plt.plot(t,Q2,'r:',linewidth=2,label=r'Heater 2 ($Q_2$)')
plt.legend(loc='best')
plt.xlabel('time (sec)')
plt.show()