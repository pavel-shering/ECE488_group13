close all; clear all; clc 

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta

res = 1000;


A = [ 0 1 0 0;
      0 0 1 0;
     -3 1 2 3;
      2 1 0 0];
  
B = [ 0 0;
      0 0;
      1 2;
      0 2];
  
C = [ 1 0 0 0;
      0 0 0 1];

D = [0 0;
     0 0];

% 1. Design a state feedback and state estimator
%% check controllaiblity STATE FEEDBACK
eig(A)
Q = [A^2*B A*B B];
rank(Q);


pSF = [-2 -3 -4 -5];
k = place(A, B, pSF);

sys_sf = ss(A-B*k, B, C ,D);

%% check observability STATE ESTIMATION
sys = ss(A,B,C,D);
Ob = obsv(sys);
R = Ob;
rank(R)

pO = pSF.*2;
F = place (A', C', pO)'

%% AUGMENTED SYSTEM, regulator

A_aug = [ A zeros(4,2);
          C zeros(2,2)]
B_aug = [B ; zeros(2,2)];

C_aug = [C zeros(2,2)];

D_aug = zeros(2,2);

eig(A_aug)
Q_aug = ctrb(A_aug, B_aug);
rank(Q_aug)


pAug = [-2 -3 -4 -5, -6, -7];
k = place(A_aug, B_aug, pAug)

k1 = k(:,1:end-2);
k2 = k(:,end-1:end);
% without estimator
A_ss = [ A-B*k1    -B*k2;
           C      zeros(2,2)];
       
% dim of B = n x #inputs (6 x 2)
B_ss = [zeros(4,2) ; - eye(2,2)];

% dim of C = #outputs x n (2 x 6)
C_ss = [C zeros(2,2)];

% dim of D = #outputs x #inputs (2 x 2)
D_ss = zeros(2,2);



G_aug = tf(ss(A_ss, B_ss, C_ss, D_ss))
G_aug = minreal(G_aug)

step(G_aug)

%% SIMULATION
%plartition k 
k1 = k(:,1:end-2);
k2 = k(:,end-1:end);

% TOTAL SYSTEM
% n = states (10)
% dimensions of A = nxn  (10x10)
A_ss = [ A       -B*k1   -B*k2;
         F*C  A-F*C-B*k1 -B*k2;
         C    zeros(2,4)  zeros(2,2)]

       
% dim of B = n x #inputs (10 x 2)
B_ss = [zeros(8,2) ; - eye(2,2)]

% dim of C = #outputs x n (2 x 10)
C_ss = [C zeros(2,6)]

% dim of D = #outputs x #inputs (2 x 2)
D_ss = zeros(2,2)

sys = ss(A_ss, B_ss, C_ss, D_ss);

figure()
step(sys)

G = tf(sys)
G = minreal(G)

figure()
step(G)


t = linspace(0,10,res);
u = ones(length(t),2);
x0 = [0 0 0 0];
x_hat0 = [0 0 0 0];
e0 = [0 0];
z0 = [x0 x_hat0 e0]; % pick any initial condition

[y,t,z] = lsim(sys, u, t, z0);

figure()
% plot(t,z(:,[5,6])) % derivative of the error
plot(t,y)
title('Step tracking showing zero steady state error');
legend('Output State 1', 'Output State 2');

