close all; clear all; clc 

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta
rng(1)

A = [ 0  1  0  0;
      0  0  1  0;
     -3  1  2  3;
      2  1  0  0];
  
B = [ 0  0;
      0  0;
      1  2;
      0  2];
  
C = [ 1  0  0  0;
      0  0  0  1];
  
D = zeros(2,2);
  
sys = ss(A, B, C, D);

%% noise
% standart deviation
sd = 0.1;
%zero mean
mu = 0;

res = 1000
t = linspace(0,10,res);
v = zeros(2,length(t));
x0 = [1 1 1 1]';
x0_hat = [0 0 0 0]';

n = sd.* randn(4, length(t))+ mu
w = sd.* randn(2, length(t))+ mu

u = [ v; n; w]
u_zero = zeros(8,length(t));

R = sd^2.*eye(2)
Q = sd^2.*eye(4)

%% STATE FEEDBACK 
% check obbservability
eig(A);
Qsf = ctrb(sys);
rank(Qsf);

pSF = [-4 -4.2 -4.4 -4.6];
k = place(A, B, pSF);

A_sf = [A-B*k]
B_sf = [B eye(4) zeros(4,2)]
C_sf = C
D_sf = [zeros(2,2) zeros(2,4) eye(2)]


sys_sf = ss(A_sf, B_sf, C_sf, D_sf);

% simulation
[y0,t,x] = lsim(sys_sf, u_zero, t, x0);
figure()
subplot(3,1,1)
plot(t,y0)

title('State Feedback initial conditions x0 [1 1 1 1]');
legend('Output 1', 'Output 2');

[y1,t,x] = lsim(sys_sf, u, t, x0);
subplot(3,1,2)
plot(t,y1)
title('State Feedback initial conditions x0 [1 1 1 1] with noise');
legend('Output 1', 'Output 2');

subplot(3,1,3)
plot(t,y0-y1)
title('Output Error');
legend('Output 1', 'Output 2');

%% STATE ESTIMATOR
% check observability
Rob = obsv(sys);
rank(Rob)

pSE = [-10 -10.1 -10.2 -10.3];
F = place (A.', C.', pSE)'


A_se = [A    -B*k;
        F*C   A-F*C-B*k]
B_se = [B eye(4) zeros(4,2);
        B zeros(4) F]
C_se = [C zeros(2,4)]
D_se = [zeros(2,2) zeros(2,4) eye(2)]


sys_se = ss(A_se, B_se, C_se, D_se);

% simulation
X0 = [x0; x0_hat]
[y2,t,x] = lsim(sys_se, u_zero, t, X0);
figure()
subplot(3,1,1)
plot(t,y2)
title('State Estimation with x0hat [0 0 0 0]');
legend('Output 1', 'Output 2');

[y3,t,x] = lsim(sys_se, u, t, X0);
subplot(3,1,2)
plot(t,y3)
title('State Estimation with x0hat [0 0 0 0] with noise');
legend('Output 1', 'Output 2');

subplot(3,1,3)
plot(t,y2-y3)
title('Output Error');
legend('Output 1', 'Output 2');

%% KALMAN FILTER
[F P ev] = lqr(A.', C.', Q, R)

F = F'

A_km = [A    -B*k;
        F*C   A-F*C-B*k]
B_km = [B eye(4) zeros(4,2);
        B zeros(4) F]
C_km = [C zeros(2,4)]
D_km = [zeros(2,2) zeros(2,4) eye(2)]

sys_km = ss(A_km, B_km, C_km, D_km);

% simulation
X0 = [x0; x0_hat]
[y4,t,x] = lsim(sys_km, u_zero, t, X0);
figure()
subplot(3,1,1)
plot(t,y4)
title('Kalman Filter with x0hat [0 0 0 0]');
legend('Output 1', 'Output 2');

[y5,t,x] = lsim(sys_km, u, t, X0);
subplot(3,1,2)
plot(t,y5)
title('Kalman Filter with x0hat [0 0 0 0] with noise');
legend('Output 1', 'Output 2');

subplot(3,1,3)
plot(t,y4-y5)
title('Output Error');
legend('Output 1', 'Output 2');


      