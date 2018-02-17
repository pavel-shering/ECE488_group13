%% LINEARIZATION
%1. Linearize about the destination Point
U = [T1 T2];
[A, B, C, D, T1, T2] = linearize_roboarm_non_optimized(A, B, C, D, U(1), U(2), Eqm_point);

%based on end effector position, need to wait for a certain amount of time
% specified in the project file, then go to the next waypoint. MAKE SURE TO
% CALCULATE THE POSITION OF THE END EFFECTOR

%% STATE FEEDBACK CONTROLLER
%2. using A,B,C,D find k s.t eig(A-Bk) in OLHP
% sys = ss(A, B, C, D)

% test eigen values of A to where the eigen values are
eig(A);

% place poles somewhere
p = [-2, -3, -4, -5];

% determine the k s.t 
k = place (A, B, p);

% test that all eig(A-Bk) in OLHP
eig(A-B*k);

%3. return u = -k * delta_x + u_op
delta_x = qout(end,:) - X0;
U = -k * delta_x + tau_0;

%{
%% CONTROLLER
% January 26th 2018
% PD controller design
k_p1 = 150;
k_p2 = 35;
k_d1 = 25;
k_d2 = 5;
K = -[k_p1, k_d1 0 0; 0 0 k_p2 k_d2];

% linear
% [t,deltaX] = ode45(@(t,deltaX)lin_roboarm(t, deltaX, A, B, (K*deltaX)), tspan, deltaX0);
% X = deltaX + Eqm_point;

if(plots)
    plot_pos(t, X(:,1), X(:,3), 'Controlled Linear');
end

X0 = [(Eqm1(1)+0.1) 0 0 0].';
% % non-linear derivative. Note that Xnl-X0 is deltaX. We have to account for
% % deltaX explicitly in nonlinar but in linear everything is already delta
% [t,Xnl] = ode45(@(t,Xnl)non_lin_roboarm(t, Xnl, K*(Xnl - Eqm_point'), rg, rm1, ...
%     rm2, rl1, rl2, rc1, rc2), tspan, X0);


if(plots)
    plot_pos(t, Xnl(:,1), Xnl(:,3), 'Controlled Non Linear');
    plot_pos(t, X(:,1) - Xnl(:,1), X(:,3) - Xnl(:,3), 'Uncontrolled Error in Position ');
end

if(vis)
    visualize(params, t, Xnl(:,1), Xnl(:,3), '');
    visualize(params, t, X(:,1), X(:,3), '');
end

% U = [0 0]'

%% NOTES:
% poles at different operating points
% at down the poles are all real and stable thus even the complex
% conjugates pairs have real part as neagtive 

% at side the poles are marginally stable, 
% ASIDE: stable impulse response is internal stability

% at top its unstable, Nothing will give you BIBO stability
%}