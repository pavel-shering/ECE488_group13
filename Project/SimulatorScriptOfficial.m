%this is the general format of the simulator script
close all; clear all; clc;

%% initializing code
% provides system parameters and and Eqm_point
Constants;
% solves for the state space system and provides S.S torques
Constants2;
flag = 0
sim_time = my_delta_t:my_delta_t:5;

%initial conditions   
X0 = x_0;
U = tau_0;

% [t,Xnl] = ode45(@(t,Xnl)non_lin_roboarm(t, Xnl, K*(Xnl - Eqm_point'), ...
%     rl1, rl2, rm1, rm2, rg, rc1, rc2), tspan, X0);

[tout,qout] = ode45(@(time,x)non_lin_roboarm(time,x,U,l1,l2,m1,m2,...
    g,c1,c2),[0 my_delta_t],X0);
% q=qout(end,[1,3])';
% q = qout(end,:)';

[m n] = size(qout);
T = m;
history_q = zeros(length(sim_time+1), 4);
history_q([1:m],:) = qout;

for t=sim_time
   T = T + 1;
   t
   %check if robot meets requirements
   
   RobotControllerScript %your script is used here.
%    if flag == 1
%        break
%    end
   [tout,qout] = ode45(@(time,x)non_lin_roboarm(time,x,U,l1,l2,m1,m2,...
       g,c1,c2),[t t+my_delta_t],qout(end,:));
   
%    q=qout(end,[1,3])';
   q = qout(end,:);
   history_q(T,:) = qout(end,:);
end
 

figure()
plot(history_q(:,[1,3]))
title('Trajectory to the first point');
legend('q1', 'q2');

 %calculate energy/time, etc...

