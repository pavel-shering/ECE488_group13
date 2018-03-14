%this is the general format of the simulator script
close all; clear all; clc;

%% initializing code
% provides system parameters and and Eqm_point
Constants;
% solves for the state space system and provides S.S torques
Constants2;

sim_time = my_delta_t:my_delta_t:2;

%initial conditions   
X0 = x_0;
U = tau_0; % init torque

[tout,qout] = ode45(@(time,x)non_lin_roboarm(time,x,U,l1,l2,m1,m2,...
    g,c1,c2),[0 my_delta_t],X0);
% q=qout(end,[1,3])';
q = qout(end,:)';
q_start = qout;
t_start = tout;

T=1;
history_q = zeros(length(sim_time), 4);
history_torques = zeros(length(sim_time),2);

for t=sim_time
   t
   %check if robot meets requirements
   
   RobotControllerScript %your script is used here.
   [tout,qout] = ode45(@(time,x)non_lin_roboarm(time,x,U,l1,l2,m1,m2,...
       g,c1,c2),[t t+my_delta_t],qout(end,:));
   
%    q=qout(end,[1,3])';
   q = qout(end,:)';
   history_q(T,:) = q;
   history_torques(T,:) = U;
   T = T + 1;
end

sim_time = [t_start' sim_time];
history_q = [q_start; history_q];

% figure()
% plot(sim_time,history_q(:,[1,3]))
% title(strcat('Trajectory from ', mat2str(X0), ' to ', mat2str(y_ref), ...
%     ' with regulator ', int2str(regulator)));
% legend('q1', 'q2');

visualize(params, sim_time', history_q(:,1), history_q(:,3), '')

 %calculate energy/time, etc...
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ENERGY CALCULATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

