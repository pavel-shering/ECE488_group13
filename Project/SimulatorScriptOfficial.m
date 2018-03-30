% [ ] 488 project work
%     * [X] Settle within 4mm for 0.5 seconds at points A,B,C,D
%         * [X] without noise
%         * [X] with noise
%     * [X] Test with  up to 10% variation in last link due to EE variations
%         * [X] mass of link 2 ,moved to link 1
%     * [X] Test with noise at joints. The friction of each joint will be changed by up to +/-0.5N/(rad/s) from your originally chosen friction values
%         * [X] TA: How do we make sure it still works under htese circumstances?
%     * [X] Ok so all 3 of those noise things happen at the same time  
%     * [ ] Report
%         * [ ] explain why our gains are good. How? wehave tunef Q. Why u tuned it. Why things are weighted certaing wayws. For speed etc

% ENERGY: 76.5326J without 10% mass stuff and noise at links 
% TIME: 3.7460s with 30 l1
% with noise and varitions l1 = 27 i get 4.1 s and energry = 60.33J
% 
%     R_kal = sd^2 .* eye(2) *1.5;
%     Q_kal = eye(4) .* ([1 2 1 2]'*1);
% 
%     R_lqr = eye(2);
%     Q_lqr = eye(4) .* [250000 1 250000 1]';
% 
%     R_lqr_aug = eye(2);
%     Q_lqr_aug = eye(6) .* [25000 1 25000 1 10000 10000]';

%this is the general format of the simulator script
close all; clear all; clc;

%% initializing code
% provides system parameters and and Eqm_point
Constants;
% solves for the state space system and provides S.S torques
Constants2;

%initial conditions   
X0 = x_0;
U = tau_0; % init torque
    
[tout,qout] = ode45(@(time,x)non_lin_roboarm(time,x,U,l1,l2,m1,m2,...
    g,c1,c2),[0 my_delta_t],X0);
q=qout(end,[1,3])';

q_start = qout(:,[1,3])';
t_start = tout;
T=1;

history_q = zeros(length(my_tspan), 2);
history_torques = zeros(length(my_tspan), 2);

for t=my_tspan
   t;
   % check if robot meets requirements
   
   RobotControllerScript %your script is used here.
   [tout,qout] = ode45(@(time,x)non_lin_roboarm(time,x,U,l1,l2,m1,m2,...
       g,c1,c2),[t t+my_delta_t],qout(end,:));
   q=qout(end,[1,3])';
   
   history_q(T,:) = q;
   history_torques(T,:) = U;
   hist_my_est(T,:) = my_delta_xhat;
   T = T + 1;
end

my_tspan1 = [t_start' my_tspan];

history_q1 = [q_start'; history_q];
visualize(my_params, my_tspan1', history_q1(:,1), history_q1(:,2), '')

% plot the true q vs estimated states
% figure()
% hold on
% plot(my_tspan', hist_my_est(:,1))
% plot(my_tspan', history_q(:,1))
% 
% figure()
% hold on
% plot(my_tspan', hist_my_est(:,3))
% plot(my_tspan', history_q(:,2))

%calculate energy/time, etc...
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ENERGY CALCULATION 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% energy = history_torques'*history_torques;
% energy = sum(sum(energy)) / 1000
my_energy

%stift solver
