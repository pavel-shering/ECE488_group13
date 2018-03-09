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

% solve for Steady State Torque
T = solve(eqns, [u1 u2]);
SST1 = subs(T.u1, [x2, x4, q1dd, q2dd], [0, 0, 0, 0]);
SST2 = subs(T.u2, [x2, x4, q1dd, q2dd], [0, 0, 0, 0]);

%formulating the state space 
y = [x1 x3];
xdot = [x(2) sol.q1dd x(4) sol.q2dd].';

%% parameter selection
% params = [l1, l2, m1, m2, g, c1, c2];

%% Equilibrium points
% arm pointing strating down, thus q1 is -pi/2
% operating point changed to pi/2 for PD controller simulation 
Eqm_point = x_0;

%% LINEAR SYSTEM
% if params change remember to recalc and copy over the A B C D and SS torque values
% into the linearize function as its optimized
xdot = subs(xdot, [sg, sm1, sm2, sl1, sl2, sc1, sc2], [g, m1, m2, l1, l2, c1, c2]);
A = jacobian(xdot, x.');
B = jacobian(xdot, u.');
C = jacobian(y, x.');
D = jacobian(y, u.');

T1 = subs(SST1, [sg, sm1, sm2, sl1, sl2, sc1, sc2], [g, m1, m2, l1, l2, c1, c2]);
T2 = subs(SST2, [sg, sm1, sm2, sl1, sl2, sc1, sc2], [g, m1, m2, l1, l2, c1, c2]);

U = [T1 T2];

[A, B, C, D, U] = linearize_roboarm_non_optimized(A, B, C, D, U, Eqm_point);
% check the location of poles if they are in OLHP // this is based on the
% equilibrium point
eig(A)

%% set the steady state torques
tau_0 = U';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONTROLLERS (linearize a bunch of controllers at diff operating points)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% res = 1000
% 
% A_aug = [ A zeros(4,2);
%           C zeros(2,2)]
% B_aug = [B ; zeros(2,2)];    
% 
% eig(A_aug)
% Q_aug = ctrb(A_aug, B_aug);
% rank(Q_aug)
% 
% 
% pAug = [-2 -3 -4 -5, -6, -7];
% k = place(A_aug, B_aug, pAug)
% 
% 
% %% SIMULATION
% %plartition k 
% k1 = k(:,1:end-2);
% k2 = k(:,end-1:end);
% 
% % n = states
% % dimensions of A = nxn  (6x6)
% A_ss = [ A-B*k1    -B*k2;
%            C      zeros(2,2)];
%        
% % dim of B = n x #inputs (6 x 2)
% B_ss = [zeros(4,2) ; - eye(2,2)];
% 
% % dim of C = #outputs x n (2 x 6)
% C_ss = [C zeros(2,2)];
% 
% % dim of D = #outputs x #inputs (2 x 2)
% D_ss = zeros(2,2);
% 
% sys = ss(A_ss, B_ss, C_ss, D_ss);
% 
% 
% t = linspace(0,10,res);
% u = ones(length(t),2);
% 
% 
% x0 = [0 0 0 0];
% e0 = [0 0];
% z0 = [x0 e0]; % pick any initial condition
% 
% [y,t,z] = lsim(sys, u, t, z0);
% 
% figure()
% % plot(t,z(:,[5,6])) % derivative of the error
% plot(t,z(:,[1,3]))
% title('Step tracking showing zero steady state error');
% legend('Output State 1', 'Output State 2');






