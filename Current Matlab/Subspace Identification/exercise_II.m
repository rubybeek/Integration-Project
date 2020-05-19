clear all
close all

load('homework.mat')

a_0=1;
b_0=0;

%% 3.1
% only the first and the fourth set are useful for identification as for
% these sets of data, the hankel matrix of the input is full rank. This
% means that te sequences u1(k) and u4(k) are persistently exciting of 
% order n=6.

n=6;
U1 = hankel((u1(1:n)),(u1(n:length(u1))));
U2 = hankel((u2(1:n)),(u2(n:length(u2))));
U3 = hankel((u3(1:n)),(u3(n:length(u3))));
U4 = hankel((u4(1:n)),(u4(n:length(u4)))); 
R1 = rank(U1);  % full rank
R2 = rank(U2);  % rank 2
R3 = rank(U3);  % rank 2
R4 = rank(U4);  % full rank

%% Exercise 1
%3.4 ARX
prompt = 'What is the desired order of the system?';
n = input(prompt);

[aest1, best1] = myarx(y1,u1,n);
[aest4, best4] = myarx(y4,u4,n);
%% Exercise 3
%rank(Hankel(u1));
prompt = 'What is the desired order of the system?';
n = input(prompt);

prompt1 = 'What is the parameter s? (s>n)';
s = input(prompt1);

[At1, Bt1, Ct1, Dt1, x0t1, S1] = mysubid(y1,u1,s,n);
[At4, Bt4, Ct4, Dt4, x0t4, S4] = mysubid(y4,u4,s,n);

[b1,a1] = ss2tf(At1,Bt1,Ct1,Dt1);
[b4,a4] = ss2tf(At4,Bt4,Ct4,Dt4);

%%
N=length(y1);

Y = hankel((y1(1:s)),[y1(s:N)' ones(1,s-1)*y1(N)]);
U = hankel((u1(1:s)),[u1(s:N)' ones(1,s-1)*u1(N)]);

r = triu(qr([U;Y]'))';

%%
R22=r((s+1:end),(s+1:2*s));
[uu,ss,vv]=svd(R22); 
semilogy(diag(ss),'xr');

%%
[Q,R] = qr([U;Y]');
r = triu(r);

%% Exercise 3

%3.5
sys_id1 = tf(best1',[1 aest1'],-1);
sys_id4 = tf(best4',[1 aest4'],-1);
[b1,a1] = ss2tf(At1,Bt1,Ct1,Dt1);
[b4,a4] = ss2tf(At4,Bt4,Ct4,Dt4);
sys_id11 = tf(b1,a1,-1);
sys_id44 = tf(b4,a4,-1);

figure 
bode(sys_id1)
hold on
bode(sys_id4)
bode(sys_id11)
bode(sys_id44)
bode(tfse);
%legend('ARX1','ARX4','subid1','subid4','original system')

%% Exercise 4

y00 = y0+02;
N=length(y0);
s=10;
Y = hankel((y0(1:s)),[y0(s:N)' ones(1,s-1)*y0(N)]);
U = hankel((u0(1:s)),[u0(s:N)' ones(1,s-1)*u0(N)]);
[Q,R]=qr([U;Y]');
r = triu(qr([U;Y]'))';
R22=r((s+1:end),(s+1:2*s));
Q2 = Q((s+1:2*s),:);
[uu,ss,vv]=svd(R22); 
semilogy(diag(ss),'xr');

Ct = uu(1,:);
At = uu(1,:)./uu(2,:);
