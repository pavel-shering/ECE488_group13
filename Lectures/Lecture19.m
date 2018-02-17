close all; clear all; clc 

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta

% observability 

A = [ 0  1;
     -2 -2];
 
B = [1 0]';

C = [1 0];

p = [-5 -6];

f = place (A', C', p);

eig( A - f'*C)

%% Deliverable

A = [ 0  0 -1;
     -1  0  0;
      3 -1 -3];
  
B = [0 0 1]';

C = [0 1 0];
D = 0;

% 1. Find the similarity transform for the observable cononical form 
%check observability
sys = ss(A,B,C,D);
Ob = obsv(sys);
R = Ob;
eig(A) % all at -1 

% solve for R_bar
A_bar = [ 0 0 -1;
          1 0 -3;
          0 1 -3];
      
C_bar = [0 0 1];

R_bar = obsv(A_bar, C_bar);
F_bar = [7 9 3]';

T = inv(R_bar) * R;
F = inv(T)*F_bar;

% 2. Design the estimator (eigen values at -2) let u = step
% x_dot_hat = (A-FC)x_hat + Fy + By
% x_dot = Ax + Bu , y = Cx

% x = [x_dot x_dot_hat]';
A_sim = [A zeros(3);
         F*C A-F*C];
     
B_sim = [B ; B];

C_sim = [C zeros(1,3)];

D_sim = 0;

sys = ss(A_sim, B_sim, C_sim, D_sim);


% 3. Choose x(0) =/ x_hat(0) and show conuligences
x0 = [0 0 0 1 1 1]; % pick any initial condition

t = linspace(0,10,100);
u = ones(length(t),1);

[y,t,x] = lsim(sys, u, t, x0);
figure()
plot(t,x(:,[1,4]))
title('Convergense of state 1 estimation to the actual state');
legend('Actual State 1', 'Estimation of state 1');

figure()
plot(t,x(:,[2,5]))
title('Convergense of state 2 estimation to the actual state');
legend('Actual State 2', 'Estimation of state 2');

figure()
plot(t,x(:,[1,4]))
title('Convergense of state 3 estimation to the actual state');
legend('Actual State 3', 'Estimation of state 3');