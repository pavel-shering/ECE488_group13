close all; clear all; clc 

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta
res = 1000

% Introduction to Optimal Control
A = [ 0  1  0  0;
      0  0  1  0;
     -3  1  2  3;
      2  1  0  0 ];
  
B = [ 0  0;
      0  0;
      1  2;
      0  2];
  
C = [ 1  0  0  0;
      0  0  0  1];

D = zeros(2,2)

% choose initial condition
x0 = [ 1 1 1 1]';
  
% Show three choices for Q and R to show differences and submit 
%% SIM 1, all states are equal and Inputs are just as valuable
Q1 = diag([1 1 1 1])
R1 = diag([1 1])

[k1 P1 ev1] = lqr(A,B,Q1,R1)

%sim
A_bar = A-B*k1

sys = ss(A_bar, B, C, D);

t = linspace(0,10,res);
v = zeros(length(t),2); % equals zero to see the term only dependent on initial conditions
[y,t,x] = lsim(sys, v, t, x0);

figure()
subplot(3,1,1)
plot(t,y)
title('Natural Response of the system (due to init conditions)');
legend('Output State 1', 'Output State 2');

subplot(3,1,2)
plot(t,x)
title('States of Natural Response of the system (due to init conditions)');
legend('State 1', 'State 2', 'State 3', 'State 4');

subplot(3,1,3)
plot(t,-k1*x')
title('Inputs to a Natural Response of the system (due to init conditions)');
legend('Input1', 'Input2');

%% SIM 2 
% push state 4 down, keep the input the same
Q2 = diag([1 1 1 100])
R2 = diag([1 1])

[k2 P2 ev2] = lqr(A,B,Q2,R2)

%sim
A_bar = A-B*k2

sys = ss(A_bar, B, C, D);

t = linspace(0,10,res);
v = zeros(length(t),2); % equals zero to see the term only dependent on initial conditions
[y,t,x] = lsim(sys, v, t, x0);

figure()
subplot(3,1,1)
plot(t,y)
title('Natural Response of the system (due to init conditions)');
legend('Output State 1', 'Output State 2');

subplot(3,1,2)
plot(t,x)
title('States of Natural Response of the system (due to init conditions)');
legend('State 1', 'State 2', 'State 3', 'State 4');

subplot(3,1,3)
plot(t,-k2*x')
title('Inputs to a Natural Response of the system (due to init conditions)');
legend('Input1', 'Input2');

%% SIM 3
% keep inputs low
Q3 = diag([1 1 1 1])
R3 = diag([1000 1000])

[k3 P3 ev3] = lqr(A,B,Q3,R3)

%sim
A_bar = A-B*k3

sys = ss(A_bar, B, C, D);

t = linspace(0,10,res);
v = zeros(length(t),2); % equals zero to see the term only dependent on initial conditions
[y,t,x] = lsim(sys, v, t, x0);

figure()
subplot(3,1,1)
plot(t,y)
title('Natural Response of the system (due to init conditions)');
legend('Output State 1', 'Output State 2');

subplot(3,1,2)
plot(t,x)
title('States of Natural Response of the system (due to init conditions)');
legend('State 1', 'State 2', 'State 3', 'State 4');

subplot(3,1,3)
plot(t,-k3*x')
title('Inputs to a Natural Response of the system (due to init conditions)');
legend('Input1', 'Input2');

cost = x0'*P3*x0
