if pull 
    %% pull
    A = A_lst(:,:,my_traj_count);
    B = B_lst(:,:,my_traj_count);
    F = F_lst(:,:,my_traj_count);
    K = K_lst(:,:,my_traj_count);
    k1 = K1_lst(:,:,my_traj_count);
    k2 = K2_lst(:,:,my_traj_count);
    U_op =  Tss_lst(:,:,my_traj_count)';
    if (abs(my_delta_xhat(1)) < 0.001 && abs(my_delta_xhat(3)) < 0.001)
    % if (abs(qout(end,1)-my_traj_angles(my_traj_count,1)) < 0.01 && abs(qout(end,3)- ...
%     my_traj_angles(my_traj_count,2)) < 0.01)
        %Forward kinematics
        tyler_x = l1 + l2*cos(q(2))*cos(q(1)) - l2*sin(q(2))*sin(q(1));
        tyler_y = l1 + l2*cos(q(2))*sin(q(1)) + l2*sin(q(2))*cos(q(1));
        if (abs(tyler_x - milestones(my_milestone_ctr, 1)) < 0.004 && abs(tyler_y - milestones(my_milestone_ctr, 2)) < 0.004)
            if ~recordedMilestoneStartTime
                milestoneStartTime = t;
                recordedMilestoneStartTime = 1;
                disp('Entered milestone for the first time')
                milestone
                disp('Starting to wait at time:')
                t
            elseif recordedMilestoneStartTime
                if (t - milestoneStartTime >= 0.5)
                    disp('Done waiting at time:')
                    t
                    % Set up future check for next milestone
                    my_milestone_ctr = my_milestone_ctr + 1
                    % reset variable and go do the torque of the next
                    % point
                    recordedMilestoneStartTime = 0;
                elseif (t - milestoneStartTime < 0.5)
                    % Output the same torque as last time
                    % and exit the script to avoid
                    % outputting torque of the next point in traj
                    U = -K * (my_delta_xhat) + U_op;
                    return
                end
            end
        end
        my_traj_count = my_traj_count+1; 
        if (my_traj_count == length(my_traj_angles)+1)
            my_traj_count = length(my_traj_angles);
        end
        my_delta_xhat = [my_traj_angles(my_traj_count-1,1) 0 my_traj_angles(my_traj_count-1,2) 0]' - ...
            [my_traj_angles(my_traj_count,1) 0 my_traj_angles(my_traj_count,2) 0]';
    end

    my_delta_xhat = (my_delta_xhat) + ( (A-F*C)*(my_delta_xhat) ...
        + B*(U-U_op) + F*(q - my_traj_angles(my_traj_count,:)')  ) * 0.001;

    % straight state feedback
    % U = -K * (qout(end,:)'-[my_traj_angles(my_traj_count,1) 0 my_traj_angles(my_traj_count,2) 0]') + U_op;

    % state estimate
    U = -K * (my_delta_xhat) + U_op;

    % regulator
%     delta_e  = my_error + (q - my_traj_angles(my_traj_count,:)') * 0.001;
%     delta_U = -k1 * my_delta_xhat - k2 * delta_e;
%     U = delta_U + Tss_lst(:,:,my_traj_count)';
else 
    %% push
    A = A_lst(:,:,my_traj_count-1);
    B = B_lst(:,:,my_traj_count-1);
    F = F_lst(:,:,my_traj_count-1);
    K = K_lst(:,:,my_traj_count-1);
    k1 = K1_lst(:,:,my_traj_count-1);
    k2 = K2_lst(:,:,my_traj_count-1);
    U_op = Tss_lst(:,:,my_traj_count-1)';
    q_op = [my_traj_angles(my_traj_count-1,1) 0 my_traj_angles(my_traj_count-1,2) 0]';
    
    
    if (abs(my_delta_xhat(1) - my_traj_angles(my_traj_count,1)) < 0.01 && ...
            abs(my_delta_xhat(3)- my_traj_angles(my_traj_count,2)) < 0.01)
    % if (abs(qout(end,1)-my_traj_angles(my_traj_count,1)) < 0.01 && abs(qout(end,3)- ...
%     my_traj_angles(my_traj_count,2)) < 0.01)
        my_traj_count = my_traj_count+1; 
        if (my_traj_count == length(my_traj_angles)+1)
            my_traj_count = length(my_traj_angles);
        end
        my_delta_xhat = [my_traj_angles(my_traj_count,1) 0 my_traj_angles(my_traj_count,2) 0]';
    end
    my_delta_xhat
    delta_y_ref =[my_traj_angles(my_traj_count,1) 0 my_traj_angles(my_traj_count,2) 0]' - q_op;
    my_delta_xhat = my_delta_xhat + ( (A-F*C)*(my_delta_xhat) ...
        + B*(U-U_op) + F*(q - delta_y_ref([1,3],:)) )*0.001;
    my_delta_xhat
    
    delta_e  = my_error + (q - delta_y_ref([1,3],:)) * 0.001;
    delta_U = -k1 * (my_delta_xhat - q_op) - k2 * delta_e;
    U = delta_U + U_op;
    
%      a = 2
end 
% push means that you will have very good accuracy for position, because
% regulator has an integrator, hovever add 2 params and increases
% complexity

% pull is simpler and perhaps faster, but not necessarily accurate. 

% use pull for intermediate waypoints and use push for finals ones

% convert angles to end effector position
% create traj points
% linearize 8 controllers
% make a decision statement for push vs pull
