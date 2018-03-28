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
    
% change of variables
eqn1 = subs(eqn1, [q1 q1d q2 q2d T1 T2], [x1 x2 x3 x4 u1 u2]);
eqn2 = subs(eqn2, [q1 q1d q2 q2d T1 T2], [x1 x2 x3 x4 u1 u2]);

% solve for q1dd and q2dd 
sol = solve([eqn1 eqn2], [q1dd q2dd]);

% solve for Steady State Torque where velocities are zero
T = solve([eqn1 eqn2], [u1 u2]);
SST1 = subs(T.u1, [x2, x4, q1dd, q2dd], [0, 0, 0, 0]);
SST2 = subs(T.u2, [x2, x4, q1dd, q2dd], [0, 0, 0, 0]);

%formulating the state space 
y = [x1 x3];
xdot = [x(2) sol.q1dd x(4) sol.q2dd].';

%% LINEAR SYSTEM
xdot = subs(xdot, [sg, sm1, sm2, sl1, sl2, sc1, sc2], [g, m1, m2, l1, l2, c1, c2]);
A_jacobian = jacobian(xdot, x.');
B_jacobian = jacobian(xdot, u.');
% C = jacobian(y, x.');
% D = jacobian(y, u.');
C = [1 0 0 0;
     0 0 1 0];

T1 = subs(SST1, [sg, sm1, sm2, sl1, sl2, sc1, sc2], [g, m1, m2, l1, l2, c1, c2]);
T2 = subs(SST2, [sg, sm1, sm2, sl1, sl2, sc1, sc2], [g, m1, m2, l1, l2, c1, c2]);

% EKF and LQR design
sd = 1/3 *pi / 180; % third of degree measurement error
mu = 0; % zero mean

R_kal =sd^2 .* eye(2);
Q_kal = eye(4) .* ([0.01 0.001 0.01 0.001]'*1000);

R_lqr = eye(2);
Q_lqr = eye(4) .* [250 1 250 1]';

R_lqr_aug = eye(2);
Q_lqr_aug = eye(6) .* [2000 1 2000 1 1 1]';

for w = 1:length(my_traj_angles)
    
    u1r = double(subs(T1, [x1 x2 x3 x4], [my_traj_angles(w,1), 0, my_traj_angles(w,2), 0]));
    u2r = double(subs(T2, [x1 x2 x3 x4], [my_traj_angles(w,1), 0, my_traj_angles(w,2), 0]));
    
    A = double(subs(A_jacobian, [x1 x2 x3 x4 u1 u2], [my_traj_angles(w,1), 0, my_traj_angles(w,2), 0, u1r, u2r]))
    B = double(subs(B_jacobian, [x1 x2 x3 x4 u1 u2], [my_traj_angles(w,1), 0, my_traj_angles(w,2), 0, u1r, u2r]))
    
    % kalman filter
    [F P_se ev_se] = lqr(A.', C.', Q_kal, R_kal)
    F = F';
    % optimal control
    [K P_sf ev_sf] = lqr(A, B, Q_lqr, R_lqr)
    
    % regulator
    A_aug = [ A zeros(4,2);
              C zeros(2,2)];
    B_aug = [B ; zeros(2,2)];
    
    [K_aug P_sf ev_sf] = lqr(A_aug, B_aug, Q_lqr_aug, R_lqr_aug);
    k1 = K_aug(:,1:end-2);
    k2 = K_aug(:,end-1:end);

    A_lst(:,:,w) = A;
    B_lst(:,:,w) = B;
    F_lst(:,:,w) = F;
    K_lst(:,:,w) = K;
    K1_lst(:,:,w) = k1;
    K2_lst(:,:,w) = k2;
    Tss_lst(:,:,w) = [u1r u2r];
end

%% set the steady state torques
tau_0 = Tss_lst(1,:,1)';

