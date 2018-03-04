%% LINEARIZATION
%1. Linearize about the destination Point
U = [T1 T2];
% [A, B, C, D, T1, T2] = linearize_roboarm_non_optimized(A, B, C, D, U(1), U(2), Eqm_point);

%based on end effector position, need to wait for a certain amount of time
% specified in the project file, then go to the next waypoint. MAKE SURE TO
% CALCULATE THE POSITION OF THE END EFFECTOR

state_feedback = 1;
state_estimator = 0;
regulator = 1;

%% STATE FEEDBACK CONTROLLER
%2. using A,B,C,D find k s.t eig(A-Bk) in OLHP
% sys = ss(A, B, C, D)
if state_feedback
    
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
    if state_feedback && ~regulator
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