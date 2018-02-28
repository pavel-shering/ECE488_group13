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
% check controllaiblity STATE FEEDBACK
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

pO = pSF.*8;
F = place (A', C', pO)';

%% 2. Simulate the conbined system for v = step input

A_com = [ A -B*k;
         F*C A-F*C-B*k];
B_com = [B; B];
C_com = [C -D*k];

D_com = D;

sys_com = ss(A_com, B_com, C_com, D_com);


%% 3. i) only state feedback
%Choose x(0) =/ x_hat(0) and show conuligences
x0 = [0 0 0 0]; % pick any initial condition

t = linspace(0,10,res);
u = ones(length(t),2);

[y,t,x] = lsim(sys_sf, u, t, x0);

figure()
plot(t,y)
title('State Feedbback only');
legend('Output 1', 'Output 2');

%%    ii) the combined system with x(0) = x_hat(0)
x0 = [0 0 0 0 0 0 0 0]; % pick any initial condition [x(0) x_hat(0)]

t = linspace(0,10,res);
u = ones(length(t),2);

[y,t,x] = lsim(sys_com, u, t, x0);
figure()
plot(t,y)
title('Combined State Feedback with State Estimation with equal initial conditions');
legend('Output 1', 'Output 2');

%% .  iii) let x(0) not eq x_hat(0)
b = 0.5 % different initial condition for x_hat(0)
x0 = [0 0 0 0 b b b b]; % pick any initial condition

t = linspace(0,10,res);
u = ones(length(t),2);

[y,t,x] = lsim(sys_com, u, t, x0);
figure()
plot(t,y)
title('Combined State Feedback with State Estimation with non equal initial conditions');
legend('Output 1', 'Output 2');



