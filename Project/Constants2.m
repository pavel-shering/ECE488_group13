%% const params
% tspan = 0:0.001:10; % set time interval
% options = odeset('RelTol',1e-5, 'AbsTol', 1e-6);

vis = 0;
plots = 0;

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

syms sg sm1 sm2 sl1 sl2 sc1 sc2

eqn1 = 0 == -T1 + [(sm1*sl1.^2)/3 + (sm2*sl2.^2)/12 + sm2*(sl1.^2 + sl2.^2/4 + sl1*sl2*cos(q2))]*q1dd + ... 
    [(sm2*sl2.^2)/3 + (sm2*sl1*sl2)/2*cos(q2)]*q2dd - ...
    sm2*sl1*sl2*sin(q2)*q1d*q2d - (sm2*sl1*sl2*sin(q2))/2*q2d.^2 + (sm1*sl1/2+sm2*sl1)*sg*cos(q1) + ...
    (sm2*sl2)/2*sg*cos(q1+q2)+sc1*q1d;

eqn2 = 0 == -T2 + [(sm2*sl2.^2)/3 + (sm2*sl1*sl2)/2*cos(q2)]*q1dd + (sm2*sl2.^2)/3*q2dd + ...
    (sm2*sl1*sl2*sin(q2))/2*q1d.^2 + (sm2*sl2)/2*sg*cos(q1+q2) + sc2*q2d;
    
eqn1 = subs(eqn1, [q1 q1d q2 q2d T1 T2], [x1 x2 x3 x4 u1 u2]);
eqn2 = subs(eqn2, [q1 q1d q2 q2d T1 T2], [x1 x2 x3 x4 u1 u2]);

% solve for q1dd and q2dd 
eqns = [eqn1 eqn2];
sol = solve(eqns, [q1dd q2dd]);

% solve for Steady State Torque where velocities are zero
T = solve(eqns, [u1 u2]);
SST1 = subs(T.u1, [x2, x4, q1dd, q2dd], [0, 0, 0, 0]);
SST2 = subs(T.u2, [x2, x4, q1dd, q2dd], [0, 0, 0, 0]);

%formulating the state space 
y = [x1 x3];
xdot = [x(2) sol.q1dd x(4) sol.q2dd].';

%% Equilibrium points
% arm pointing strating down, thus q1 is -pi/2
% operating point changed to pi/2 for PD controller simulation this is x_0

%% LINEAR SYSTEM
% if params change remember to recalc and copy over the A B C D and SS torque values
% into the linearize function as its optimized
xdot = subs(xdot, [sg, sm1, sm2, sl1, sl2, sc1, sc2], [g, m1, m2, l1, l2, c1, c2]);
A = jacobian(xdot, x.')
B = jacobian(xdot, u.')
C = jacobian(y, x.')
D = jacobian(y, u.')

T1 = subs(SST1, [sg, sm1, sm2, sl1, sl2, sc1, sc2], [g, m1, m2, l1, l2, c1, c2]);
T2 = subs(SST2, [sg, sm1, sm2, sl1, sl2, sc1, sc2], [g, m1, m2, l1, l2, c1, c2]);
U = [T1 T2]

[A, B, C, D, U] = linearize_roboarm_non_optimized(A, B, C, D, U, x_0)
% check the location of poles if they are in OLHP // this is based on the
% equilibrium point
% eig(A)

%% set the steady state torques
tau_0 = U';




