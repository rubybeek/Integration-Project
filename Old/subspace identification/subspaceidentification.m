
clearvars
close all
load('data.1.mat')



% Subspace identification

y1 = data(:,1);
y2 = data(:,2);
y = [y1
     y2];
 
u1 = data(:,3);
u2 = data(:,4);

u = [u1
     u2];
 


n = 2;

s = 3 * n;

[At1, Bt1, Ct1, Dt1, x0t1, S1]=mysubid(y1 ,u1 ,s ,n);
[At12, Bt12, Ct12, Dt12, x0t12, S12]=mysubid(y1 ,u2 ,s ,n);
[At21, Bt21, Ct21, Dt21, x0t21, S21]=mysubid(y2 ,u1 ,s ,n);
[At2, Bt2, Ct2, Dt2, x0t2, S2]=mysubid(y2 ,u2 ,s ,n);
[At, Bt, Ct, Dt, x0t, S]=mysubid(y ,u ,s ,n);

% 
% At = [At1 At12
%       At21 At2]
  

%%
[num1, den1] = ss2tf(At1,Bt1,Ct1,Dt1);

tf_system1 = num1/den1;

discrete_system1 = c2d(tf_system1);

x1 = [];

for k = 1:399

    x1(k+1) = At1 * x1(k) + Bt1 * u1(k);
   
end
