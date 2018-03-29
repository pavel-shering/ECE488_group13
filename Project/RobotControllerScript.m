mu = [0 0];
sigma = [1/3 1/3] * pi / 180;
noise = mvnrnd(mu,sigma.^2)';

% noise = sd.*randn(2,1);
if noise(1) > 1 
   noise = [1 noise(2)]'
end
if noise(2) > 1 
   noise = [noise(1) 1]'
end
q=q + noise;


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
        my_traj_count = my_traj_count+1; 
        if (my_traj_count == length(my_traj_angles)+1)
            my_traj_count = length(my_traj_angles);
        else 
            my_delta_xhat = [my_traj_angles(my_traj_count-1,1) 0 my_traj_angles(my_traj_count-1,2) 0]' - ...
            [my_traj_angles(my_traj_count,1) 0 my_traj_angles(my_traj_count,2) 0]';
        end
    end

    my_delta_xhat = (my_delta_xhat) + ( (A-F*C)*(my_delta_xhat) ...
        + B*(U-U_op) + F*(q - my_traj_angles(my_traj_count,:)')  ) * 0.001;

    % straight state feedback
    % U = -K * (qout(end,:)'-[my_traj_angles(my_traj_count,1) 0 my_traj_angles(my_traj_count,2) 0]') + U_op;

    % state estimate
    U = -K * (my_delta_xhat) + U_op;

    % regulator
    my_error  = my_error + (q - my_traj_angles(my_traj_count,:)') * 0.001;
    delta_U = -k1 * my_delta_xhat - k2 * my_error;
    U = delta_U + U_op;
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
    q_dest = [my_traj_angles(my_traj_count,1) 0 my_traj_angles(my_traj_count,2) 0]';
    
%     delta_y_ref =q_dest - q_op;
    my_delta_xhat = (my_delta_xhat)  + ( (A-F*C)*(my_delta_xhat) ...
        + B*(U-U_op) + F*(q  - q_op([1,3],:)) )*0.001;    
    
    %     U = -K * (my_delta_xhat) + U_op;
    
    my_error = my_error + (q - q_dest([1,3],:)) * 0.001;
    delta_U = -k1 * (my_delta_xhat) - k2 * my_error;
    U = delta_U + U_op;
    
    cur_pos = my_delta_xhat + q_op;
    X_pred = l1 * cos(cur_pos(1)) + l2 * cos(cur_pos(1) + cur_pos(3)); % compute x coordinates
    Y_pred = l1 * sin(cur_pos(1)) + l2 * sin(cur_pos(1) + cur_pos(3));
    
    X = l1 * cos(my_traj_angles(my_traj_count,1)) + l2 * cos(my_traj_angles(my_traj_count,1) + my_traj_angles(my_traj_count,2));
    Y = l1 * sin(my_traj_angles(my_traj_count,1)) + l2 * sin(my_traj_angles(my_traj_count,1) + my_traj_angles(my_traj_count,2));
     
    
    if (sqrt((X_pred - X)^2 + (Y_pred - Y)^2) <= 0.004)
%     if (abs(my_delta_xhat(1) - my_traj_angles(my_traj_count-1,1)) < 0.01 && ...
%             abs(my_delta_xhat(3)- my_traj_angles(my_traj_count-1,2)) < 0.01)
%     if (abs(qout(end,1)-my_traj_angles(my_traj_count,1)) < 0.01 && abs(qout(end,3)- ...
%     my_traj_angles(my_traj_count,2)) < 0.01)
        my_traj_count = my_traj_count+1; 
        if (my_traj_count == length(my_traj_angles)+1)
            my_traj_count = length(my_traj_angles);
        else 
            my_delta_xhat = my_delta_xhat + q_op - q_dest;
        end

    end

end 
% push means that you will have very good accuracy for position, because
% regulator has an integrator, however add 2 params and increases
% complexity

% pull is simpler and perhaps faster, but not necessarily accurate. 

% use pull for intermediate waypoints and use push for finals ones
