%% LINEARIZATION
%1. Linearize about the destination Point
U = [T1 T2];
% [A, B, C, D, T1, T2] = linearize_roboarm_non_optimized(A, B, C, D, U(1), U(2), Eqm_point);

%based on end effector position, need to wait for a certain amount of time
% specified in the project file, then go to the next waypoint. MAKE SURE TO
% CALCULATE THE POSITION OF THE END EFFECTOR

statefeedback = 1;
regulator = 1;

%% STATE FEEDBACK CONTROLLER
%2. using A,B,C,D find k s.t eig(A-Bk) in OLHP
% sys = ss(A, B, C, D)
if statefeedback
    
    % test eigen values of A to where the eigen values are
    eig(A);

    % check controlability
    Q = ctrb(A, B);
    rank(Q);

    % place poles somewhere
    p = 100*[-1, -1.1, -8, -8.1];

    % determine the k s.t 
    k = place(A, B, p);

    % test that all eig(A-Bk) in OLHP
    eig(A-B*k);

    %3. return u = -k * delta_x + u_op
%     delta_x = qout(end,:) - X0
    y_ref(:,my_traj)
    qout(end,:)
    
    delta_x = qout(end,:)' - y_ref(:,my_traj)
    if statefeedback && ~regulator
        U = -k * delta_x + tau_0
    end
    % U=-K*(qout(end,:)'-x_des)+U_eq
end
    
%% REGULATOR
% AUGMENTED SYSTEM
if regulator
    A_aug = [ A zeros(4,2);
              C zeros(2,2)];
    B_aug = [B ; zeros(2,2)];    

    eig(A_aug)

    Q_aug = ctrb(A_aug, B_aug);
    rank(Q_aug);

    % pAug = 100*[-2 -3 -4 -5, -6, -7];
    pAug = 100*[-1 -1.5 -2 -2.5, -3, -3.5];
    pAug = 10*[-1.1 -1.2 -8.3 -8.4, -8.5, -8.6];
    k = place(A_aug, B_aug, pAug);

    eig(A_aug - B_aug*k);

    %partition k 
    k1 = k(:,1:end-2);
    k2 = k(:,end-1:end);

    % return U
    % if (abs(qout(end,[1,3])' -  y_ref(:,my_traj)) <= my_traj_threshold)
    %     my_traj = my_traj+1;
    %     flag = 1
    %     % Change tau_0
    % end

    % delta_e  = e_0 + (qout(end,[1,3])' - y_ref(:,my_traj)) * my_delta_t;

    delta_e  = e_0 + (qout(end,[1,3])' - y_ref([1,3],my_traj)) * my_delta_t;

    U = -k1 * delta_x - k2 * delta_e + tau_0;
    e_0 = delta_e;
end

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