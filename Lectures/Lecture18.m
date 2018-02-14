close all; clear all; clc 

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta

% We sent a problem to another team

%% solve for Q
A = [-1    -1    -1;
      1    -2    -1;
     -1     1     0];

B = [1 1 0]';

C = [1 1 0];
D = 0;

sys_old = ss(A, B, C, D);

% test eigen values of A to make sure they are all at -1
eig(A)

% eig(A)
Q = [A^2*B A*B B];
rank(Q)

%% solve for Q_bar
A_bar = [ 0  1  0;
          0  0  1;
         -1 -3 -3];
  
B_bar = [0 0 1]';

k_bar = [7 9 3]; % its a gain matrix that scales the
% states to act like a controller to be used for the input

% test that the A and A_bar have the same eigen values
eig(A)-eig(A_bar);

% find Q_bar
Q_bar = [A_bar^2*B_bar A_bar*B_bar B_bar];
rank(Q_bar) % this has to be full 

%% place all poles at -2, 
% find T matrix to place in controllable cononical form
T = Q_bar * inv(Q);

% Deliver the 'k' and show step response
k = k_bar * T;

% do a check that the eigen values are at all at -2
eig(A -  B*k);

%% step response u = -kx
C = [1 1 0];
D = 0;

% formulate the new system
% x_dot = Ax + Bu where u = -kx + v
% x_dot = (A-B*k)x + Bv
A_new = A-B*k;
sys = ss(A_new, B, C, D);

t = linspace(0,10,100);
u = ones(length(t),1);
figure() 
lsim(sys_old, sys, u, t);
title('Step Response of old vs new');
legend('Old system with eig vals at -1', 'New system with eig vals at -2');

%% internally stable is always BIBO stable, not vice versa
% [y,t,x] = lsim(sys, u, t)
% looking at states vs the output
% figure()
% plot(t,x)
% figure()
% plot(t,y)
