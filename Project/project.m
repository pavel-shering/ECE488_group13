close all; clear all; clc;

%model variables
syms q1 q1d q1dd T1
syms q2 q2d q2dd T2

syms x1 x2 u1 %q1
syms x3 x4 u2 %q2

g = 9.81; % [m/s^2]
m1 = 5 ; %[kg]
m2 = 5 ; %[kg]

l1 = 1; %[m]
l2 = 1; %[m]

%dampers
c1 = 0.5; 
c2 = 0.5;

params = [m1 m2 l1 l2 c1 c2];

% [q1 q1d q2 q2d]
x = [x1 x2 x3 x4];
% [T1 T2]
u = [u1 u2];

eqn1 = T1 == [(m1*l1.^2)/3 + (m2*l2.^2)/12 + m2*(l1.^2 + l2.^2/4 + l1*l2*cos(q2))]*q1dd + ... 
    [(m2*l2.^2)/3 + (m2*l1*l2)/2*cos(q2)]*q2dd - ...
    m2*l1*l2*sin(q2)*q1d*q2d - (m2*l1*l2*sin(q2))/2*q2d.^2 + (m1*l1/2+m2*l1)*g*cos(q1) + ...
    (m2*l2)/2*g*cos(q1+q2)+c1*q1d;

eqn2 = T2 == [(m2*l2.^2)/3 + (m2*l1*l2)/2*cos(q2)]*q1dd + (m2*l2.^2)/3*q2dd + ...
    (m2*l1*l2*sin(q2))/2*q1d.^2 + (m2*l2)/2*g*cos(q1+q2) + c2*q2d;
    
eqn1 = subs(eqn1, [q1 q1d q2 q2d T1 T2], [x1 x2 x3 x4 u1 u2])
eqn2 = subs(eqn2, [q1 q1d q2 q2d T1 T2], [x1 x2 x3 x4 u1 u2])

Eqm1 = [-pi/2 0];
u1_0 = 0;

Eqm2 = [0 0];
u2_0 = 0;

Eqm_tot = [Eqm1 Eqm2];

%solve for q1dd and q2dd 
eqns = [eqn1 eqn2];
sol = solve(eqns, [q1dd q2dd])

%formulating the state space 
y = [x1 x3];
xdot = [x(2) sol.q1dd x(4) sol.q2dd].'

A = jacobian(xdot, x.');
B = jacobian(xdot, u.');
C = jacobian(y, x.');
D = jacobian(y, u.');

% evaluate at equlibrium points
A = double(subs(A, [x, u], [Eqm1 Eqm2 u1_0  u2_0]));
B = double(subs(B, [x, u], [Eqm1 Eqm2 u1_0  u2_0]));
C = double(subs(C, [x, u], [Eqm1 Eqm2 u1_0  u2_0]));
D = double(subs(D, [x, u], [Eqm1 Eqm2 u1_0  u2_0]));

% xdot = A*x + B*u
% y = C*X + D*u

%% Initial condotions
tspan = 0:0.001:5; % set time interval

% initial conditions
T1 = 0;
T2 = 0;
T = [T1 T2].';
X0 = [-pi/2 0 0 0];
linX0 = [-pi/2 0 0 0];

%linear derivative
[t,X] = ode45(@(t,X)lin_roboarm(t, X, A, B, T), tspan, linX0);
deltaX = X; %+ Eqm_tot;

% non-linear derivative
[t,Xnl] = ode45(@(t,Xnl)non_lin_roboarm(t, Xnl, T), tspan, X0);


% figure(1)
% hold on 
visualize(params, t, Xnl(:,1), Xnl(:,3), '')
visualize(params, t, deltaX(:,1), deltaX(:,3), '')

%{
plot(t,deltaX(:,1));
plot(t,Xnl(:,1));
xlabel('time');
ylabel('angle q1');
legend('Linearized', 'Non-Linearized')

figure(2)
hold on
plot(t,deltaX(:,3));
plot(t,Xnl(:,3));
xlabel('time');
ylabel('angle q2');
legend('Linearized', 'Non-Linearized')

%}
