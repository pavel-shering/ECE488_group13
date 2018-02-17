%this is the general format of the simulator script
close all; clear all; clc;

%% initializing code
% provides system parameters and and Eqm_point
Constants;
% solves for the state space system and provides S.S torques
Constants2;


%initial conditions   
X0 = x_0;
U = tau_0;


[tout,qout] = ode45(@(time,x)non_lin_roboarm(time,x,U,l1,l2,m1,m2,g,c1,c2),[0 0.001],X0);
% q=qout(end,[1,3])';
% history_qout = % of all 4 states
for t=0.001:0.001:10

   %check if robot meets requirements

   RobotControllerScript %your script is used here.
  
   [tout,qout] = ode45(@(time,x)non_lin_roboarm(time,x,U,l1,l2,m1,m2,g,c1,c2),[t t+0.001],qout(end,:));
%    q=qout(end,[1,3])';
   
end
 
 %calculate energy/time, etc...

