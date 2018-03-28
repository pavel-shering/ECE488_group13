% state estimator with state feedback
% delta_x_hat = ( (A_lst(:,:,my_traj_count) - F_lst(:,:,my_traj_count) * C)*my_state_estimate_vector ...
%     + B_lst(:,:,my_traj_count)*(U - Tss_lst(:,:,my_traj_count)') + F_lst(:,:,my_traj_count)*(q - x_0([1,3])) )*my_delta_t ...
%     + my_state_estimate_vector;
% 
% my_state_estimate_vector = delta_x_hat;

% if (abs(delta_x_hat(1) - my_traj_angles(my_traj_count,1)) < 0.05 && abs(delta_x_hat(3)- my_traj_angles(my_traj_count,2)) < 0.05)
%     my_traj_count = my_traj_count+1; 
%     if (my_traj_count == length(my_traj_angles)+1)
%         my_traj_count = length(my_traj_angles);
%     end
% end
% U = -K_lst(:,:,my_traj_count)* my_state_estimate_vector + Tss_lst(:,:,my_traj_count)';


% x_hat_old = my_state_estimate_vector;
A = A_lst(:,:,my_traj_count);
B = B_lst(:,:,my_traj_count);
F = F_lst(:,:,my_traj_count);
K = K_lst(:,:,my_traj_count);
U_op =  Tss_lst(:,:,my_traj_count)';
% x_des =[ my_traj_angles(my_traj_count-1,1) 0 my_traj_angles(my_traj_count-1,2) 0]';



% if (abs(my_state_estimate_vector(1)) < 0.01 && abs(my_state_estimate_vector(3)) < 0.01)

if (abs(qout(end,1)-my_traj_angles(my_traj_count,1)) < 0.01 && abs(qout(end,3)-my_traj_angles(my_traj_count,2)) < 0.01)
    my_traj_count = my_traj_count+1; 
    if (my_traj_count == length(my_traj_angles)+1)
        my_traj_count = length(my_traj_angles);
    end
    my_state_estimate_vector = [my_traj_angles(my_traj_count-1,1) 0 my_traj_angles(my_traj_count-1,2) 0]' - ...
        [my_traj_angles(my_traj_count,1) 0 my_traj_angles(my_traj_count,2) 0]';
end

% my_state_estimate_vector = (my_state_estimate_vector) + ( (A-F*C)*(my_state_estimate_vector) ...
%     + B*(U-U_op) + F*(q - my_traj_angles(my_traj_count,:)')  )*0.001;

U = -K * (qout(end,:)'-[my_traj_angles(my_traj_count,1) 0 my_traj_angles(my_traj_count,2) 0]') + U_op;

% U = -K * (my_state_estimate_vector) + U_op;



% delta_x_hat = ( (A_lst(:,:,my_traj_count) - F_lst(:,:,my_traj_count) * C)*my_state_estimate_vector ...
%     + B_lst(:,:,my_traj_count)*(U - Tss_lst(:,:,my_traj_count)') + F_lst(:,:,my_traj_count)*(q - my_traj_angles(my_traj_count,:)') )*my_delta_t ...
%     + my_state_estimate_vector;

% delta_x_hat = ( (A_lst(:,:,my_traj_count) - F_lst(:,:,my_traj_count) * C)*my_state_estimate_vector ...
%     + B_lst(:,:,my_traj_count)*(U) + F_lst(:,:,my_traj_count)*(q) )*my_delta_t ...
%     + my_state_estimate_vector;

% my_state_estimate_vector = delta_x_hat;

% if (abs(delta_x_hat(1) - my_traj_angles(my_traj_count,1)) < 0.05 && abs(delta_x_hat(3)- my_traj_angles(my_traj_count,2)) < 0.05)
%     my_traj_count = my_traj_count+1; 
%     if (my_traj_count == length(my_traj_angles)+1)
%         my_traj_count = length(my_traj_angles);
%     end
% end

% U = -K_lst(:,:,my_traj_count)* my_state_estimate_vector + Tss_lst(:,:,my_traj_count)';

% regulator
% A_aug = [ A zeros(4,2);
%           C zeros(2,2)];
% B_aug = [B ; zeros(2,2)];
% 
% [K_aug P_sf ev_sf] = lqr(A_aug, B_aug, Q_lqr_aug, R_lqr_aug);
% k1 = K_aug(:,1:end-2);
% k2 = K_aug(:,end-1:end);
% 
% delta_e  = my_error + (q - my_traj_angles(my_traj_count,:)') * my_delta_t;
% delta_U = -k1 * delta_x_hat - k2 * delta_e;
% U = delta_U + Tss_lst(:,:,my_traj_count)';
% my_error = delta_e;


% push means that you will have very good accuracy for position, because
% regulator has an integrator, hovever add 2 params and increases
% complexity

% pull is simpler and perhaps faster, but not necessarily accurate. 

% use pull for intermediate waypoints and use push for finals ones


% convert angles to end effector position
% create traj points
% linearize 8 controllers
% make a decision statement for push vs pull



%{
state_feedback = 1;
state_estimator = 1;
regulator = 1;
kalman_filter = 1;
optimal_control = 1;
rng(1)

% EKF and LQR design
sd = pi/180 *1/3; % third of degree measurement error
mu = 0; % zero mean

R_kal =sd^2.* eye(2);
    
Q_kal = eye(4) .* ([0.01 0.01 0.01 0.01]');

R_lqr = eye(2);
    
Q_lqr = eye(4) .* [100 1 100 1]';


A = A_lst(:,:,my_traj_count);
B = B_lst(:,:,my_traj_count);



%% STATE FEEDBACK CONTROLLER
if state_feedback
    
    % test eigen values of A to where the eigen values are
    eig(A);

    % check controlability
    Qctr = ctrb(A, B);
    rank(Qctr);

    
    pSF = 100*[-1, -1.1, -8, -8.1];
    if optimal_control
       [k P_sf ev_sf] = lqr(A, B, Q_lqr, R_lqr);
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
%         sys = ss(A,B,C,D);
%         Rob = obsv(sys);
%         rank(Rob);
        
        if kalman_filter
           [F P_se ev_se] = lqr(A.', C.', Q_kal, R_kal);
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
    
    delta_x = q - x_0([1,3]);
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
%     pAug = 10*[-1.1 -1.2 -8.3 -8.4, -8.5, -8.6];
%     k = place(A_aug, B_aug, pAug);
    [k P_sf ev_sf] = lqr(A_aug, B_aug, Q_lqr_aug, R_lqr);

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

%}
 