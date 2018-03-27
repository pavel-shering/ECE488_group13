%% LINEARIZATION
%1. Linearize about the destination Point
% [A, B, C, D, U] = linearize_roboarm_non_optimized(A, B, C, D, U, Eqm_point);

%based on end effector position, need to wait for a certain amount of time
% specified in the project file, then go to the next waypoint. MAKE SURE TO
% CALCULATE THE POSITION OF THE END EFFECTOR

state_feedback = 1;
state_estimator = 1;
regulator = 1;
kalman_filter = 1;
optimal_control = 1;
rng(1)

% EKF and LQR design
sd = pi/180 *1/3; % third of degree measurement error
mu = 0; % zero mean

% w = sd.* randn(2, length(t))+ mu;

% play around with modelling error due to linearization
% n = sd.* randn(4, length(t))+ mu;

R = sd^2.* [1 0 
            0 1];
    
Q = sd^2.* [100000 0 0 0;
            0 1 0 0;
            0 0 100000 0;
            0 0 0 1];


%% STATE FEEDBACK CONTROLLER
if state_feedback
    
    % test eigen values of A to where the eigen values are
    eig(A);

    % check controlability
    Qctr = ctrb(A, B);
    rank(Qctr);

    
    pSF = 100*[-1, -1.1, -8, -8.1];
    if optimal_control
       [k P_sf ev_sf] = lqr(A, B, Q, R);
    else
        % place poles somewhere
        % determine the k s.t 
        k = place(A, B, pSF);
    end


    % test that all eig(A-Bk) in OLHP
    eig(A-B*k);
    
    %% ESTIMATOR
    if state_estimator

        %check observerbility
        sys = ss(A,B,C,D);
        Rob = obsv(sys);
        rank(Rob);
        
        if kalman_filter
           [F P_se ev_se] = lqr(A.', C.', Q, R);
            F = F';
        else 
            pO = pSF.*2;
            F = place (A', C', pO)';
        end
        
%         delta_x_hat = ( (A-F*C)*my_state_estimate_vector + B*(U-tau_0) + F*(q([1,3])-x_0([1,3])) )*my_delta_t ...
%             + my_state_estimate_vector;
        delta_x_hat = ( (A - F * C)*my_state_estimate_vector + B*(U - tau_0) + F*(q - x_0([1,3])) )*my_delta_t ...
            + my_state_estimate_vector;
        my_state_estimate_vector = delta_x_hat;
    end
    
    delta_x = q - x_0(end,:);
    if state_feedback && ~regulator && ~state_estimator
        delta_U = -k * delta_x;
        U = delta_U + tau_0;
    elseif state_feedback && ~regulator && state_estimator
        delta_U = -k * delta_x_hat;
        U = delta_U + tau_0;
    end
    
end
    
%% REGULATOR
% AUGMENTED SYSTEM
if regulator
    A_aug = [ A zeros(4,2);
              C zeros(2,2)];
    B_aug = [B ; zeros(2,2)];    

    eig(A_aug);

    Q_aug = ctrb(A_aug, B_aug);
    rank(Q_aug);

    % pAug = 100*[-2 -3 -4 -5, -6, -7];
%     pAug = 100*[-1 -1.5 -2 -2.5, -3, -3.5];
    pAug = 10*[-1.1 -1.2 -8.3 -8.4, -8.5, -8.6];
    k = place(A_aug, B_aug, pAug);

    eig(A_aug - B_aug*k);

    %partition k 
    k1 = k(:,1:end-2);
    k2 = k(:,end-1:end);
    
    % return U
    % if (abs(qout(end,[1,3])' -  my_traj_angles(my_traj_count,:)') <= my_traj_threshold)
    %     my_traj_count = my_traj_count+1;
    %     % Change tau_0
    % end
    
    if ~state_estimator
%         delta_e  = my_error + (qout(end,[1,3])' - my_traj_angles(my_traj_count)') * my_delta_t;
        delta_e  = my_error + (q - my_traj_angles(my_traj_count,:)') * my_delta_t;
        delta_U = -k1 * delta_x - k2 * delta_e;
        U = delta_U + tau_0;
        my_error = delta_e;
    elseif state_estimator
%         delta_e  = my_error + (qout(end,[1,3])' - my_traj_angles(:,my_traj_count)) * my_delta_t;
        delta_e  = my_error + (q - my_traj_angles(my_traj_count,:)') * my_delta_t;
        delta_U = -k1 * delta_x_hat - k2 * delta_e;
        U = delta_U + tau_0;
        my_error = delta_e;
    end
   
end



% if (abs(x_hat(1)) < 0.05 && abs(x_hat(3)) < 0.05)
%     j = j+1; 
%     if (j == 32)
%         j = 31;
%     end
% end


% push means that you will have very good accuracy for position, because
% regulator has an integrator, hovever add 2 params and increases
% complexity

% pull is simpler and perhaps faster, but not necessarily accurate. 

% use pull for intermediate waypoints and use push for finals ones


% convert angles to end effector position
% create traj points
% linearize 8 controllers
% make a decision statement for push vs pull

    