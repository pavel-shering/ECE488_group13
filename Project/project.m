close all; clearvars; clc;

%% const params
tspan = 0:0.001:10; % set time interval
options = odeset('RelTol',1e-5, 'AbsTol', 1e-6);

vis = 0;
plots = 1;

%% TOTAL SYSTEM
% symbolic model variables
syms q1 q1d q1dd T1
syms q2 q2d q2dd T2

syms x1 x2 u1 %q1
syms x3 x4 u2 %q2

% change of variables
%   [q1 q1d q2 q2d]
x = [x1 x2 x3 x4];
%   [T1 T2]
u = [u1 u2];

syms g m1 m2 l1 l2 c1 c2

eqn1 = 0 == -T1 + [(m1*l1.^2)/3 + (m2*l2.^2)/12 + m2*(l1.^2 + l2.^2/4 + l1*l2*cos(q2))]*q1dd + ... 
    [(m2*l2.^2)/3 + (m2*l1*l2)/2*cos(q2)]*q2dd - ...
    m2*l1*l2*sin(q2)*q1d*q2d - (m2*l1*l2*sin(q2))/2*q2d.^2 + (m1*l1/2+m2*l1)*g*cos(q1) + ...
    (m2*l2)/2*g*cos(q1+q2)+c1*q1d;

eqn2 = 0 == -T2 + [(m2*l2.^2)/3 + (m2*l1*l2)/2*cos(q2)]*q1dd + (m2*l2.^2)/3*q2dd + ...
    (m2*l1*l2*sin(q2))/2*q1d.^2 + (m2*l2)/2*g*cos(q1+q2) + c2*q2d;
    
eqn1 = subs(eqn1, [q1 q1d q2 q2d T1 T2], [x1 x2 x3 x4 u1 u2]);
eqn2 = subs(eqn2, [q1 q1d q2 q2d T1 T2], [x1 x2 x3 x4 u1 u2]);

% solve for q1dd and q2dd 
eqns = [eqn1 eqn2];
sol = solve(eqns, [q1dd q2dd]);

% solve for Steady State Torque
T = solve(eqns, [u1 u2]);
SST1 = subs(T.u1, [x2, x4, q1dd, q2dd], [0, 0, 0, 0]);
SST2 = subs(T.u2, [x2, x4, q1dd, q2dd], [0, 0, 0, 0]);

%formulating the state space 
y = [x1 x3];
xdot = [x(2) sol.q1dd x(4) sol.q2dd].';

%% parameter selection
rg = 9.81; % [m/s^2]
% masses of links
rm1 = 1.0 ; %[kg]
rm2 = 1.0 ; %[kg]

% link lengths
rl1 = 1.0; %[m]
rl2 = 1.0; %[m]

% dampers
rc1 = 1.0; 
rc2 = 1.0;

params = [rg, rm1, rm2, rl1, rl2, rc1, rc2];

%% Equilibrium points
% arm pointing strating down, thus q1 is -pi/2
% operating point changed to pi/2 for PD controller simulation 
Eqm1 = [-pi/2 0];
u1_0 = 0;

% q2 relative to q1 is zero 
Eqm2 = [0 0];
u2_0 = 0;

Eqm_point = [Eqm1 Eqm2];

%% LINEAR SYSTEM
% if params change remember to recalc and copy over the A B C D and SS torque values
% into the linearize function as its optimized
xdot = subs(xdot, [g, m1, m2, l1, l2, c1, c2], [rg, rm1, rm2, rl1, rl2, rc1, rc2]);
% A = jacobian(xdot, x.');
% B = jacobian(xdot, u.');
% C = jacobian(y, x.');
% D = jacobian(y, u.');
% 
% T1 = subs(SST1, [g, m1, m2, l1, l2, c1, c2], [rg, rm1, rm2, rl1, rl2, rc1, rc2]);
% T2 = subs(SST2, [g, m1, m2, l1, l2, c1, c2], [rg, rm1, rm2, rl1, rl2, rc1, rc2]);

[A, B, C, D, T1, T2] = linearize_roboarm(Eqm_point);

% check the location of poles if they are in OLHP // this is based on the
% equilibrium point
% eig(A)

%% plotting of non linear system
% initial state
X0 = [(-pi/2+0.1) 0 0 0];
T = [T1 T2];
% non-linear derivative  T, l1, l2, m1, m2, g, c1, c2)
[t,Xnl] = ode45(@(t,Xnl)non_lin_roboarm(t, Xnl, T, rl1, rl2, rm1, rm2, rg, rc1, rc2), tspan, X0, options);

if(plots)
    plot_pos(t, Xnl(:,1), Xnl(:,3), 'Uncontrolled Non Linear');
end

%% poles of the linear system at certain Eqm points
% xdot = A*x + B*u
% y = C*X + D*u
s = tf('s');
TF = ss(A,B,C,D);
TFpoles = pole(TF);

%% plotting linear system
% this is an initial condition about the operating point for the linearized
% equation. Thus for deltaX0 = 0 means no disturbance, since nothing is
% added to the Eqm point of -pi/2
deltaX0 = [0.1 0 0 0];

%linear derivative
[t,deltaX] = ode45(@(t,deltaX)lin_roboarm(t, deltaX, A, B, T'), tspan, deltaX0, options);
X = deltaX + Eqm_point;

if(plots)
    plot_pos(t, X(:,1), X(:,3), 'Uncontrolled Linear');
    plot_pos(t, X(:,1) - Xnl(:,1), X(:,3) - Xnl(:,3), 'Uncontrolled Error in Position ');
end
if(vis)
    visualize(params, t, Xnl(:,1), Xnl(:,3), '');
    visualize(params, t, X(:,1), X(:,3), '');
end

%% CONTROLLER
% January 26th 2018
% PD controller design
k_p1 = 150;
k_p2 = 35;
k_d1 = 25;
k_d2 = 5;
K = -[k_p1, k_d1 0 0; 0 0 k_p2 k_d2];

% linear
[t,deltaX] = ode45(@(t,deltaX)lin_roboarm(t, deltaX, A, B, (K*deltaX)), tspan, deltaX0);
X = deltaX + Eqm_point;

if(plots)
    plot_pos(t, X(:,1), X(:,3), 'Controlled Linear');
end

X0 = [(Eqm1(1)+0.1) 0 0 0].';
% % non-linear derivative. Note that Xnl-X0 is deltaX. We have to account for
% % deltaX explicitly in nonlinar but in linear everything is already delta
[t,Xnl] = ode45(@(t,Xnl)non_lin_roboarm(t, Xnl, K*(Xnl - Eqm_point'), ...
    rl1, rl2, rm1, rm2, rg, rc1, rc2), tspan, X0);


if(plots)
    plot_pos(t, Xnl(:,1), Xnl(:,3), 'Controlled Non Linear');
    plot_pos(t, X(:,1) - Xnl(:,1), X(:,3) - Xnl(:,3), 'Uncontrolled Error in Position ');
end

if(vis)
    visualize(params, t, Xnl(:,1), Xnl(:,3), '');
    visualize(params, t, X(:,1), X(:,3), '');
end

%% NOTES:
% poles at different operating points
% at down the poles are all real and stable thus even the complex
% conjugates pairs have real part as neagtive 

% at side the poles are marginally stable, 
% ASIDE: stable impulse response is internal stability

% at top its unstable, Nothing will give you BIBO stability