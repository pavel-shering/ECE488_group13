%put constant values in this file%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %You NEED these constants
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% l1 = sqrt(20^2 + 20^2)/2; %link 1 length
% l2 = sqrt(20^2 + 20^2)/2; %link 2 length
% m1 = l1*1.4; %link 1 mass
% m2 = l1*1.4; %link 2 mass
% c1 = 6; % damping of link 1
% c2 = 6; % damping of link 2
% g = 3.7; %acceleration due to gravity m/s^2 on mars

l1 = 1; %link 1 length
l2 = 1; %link 2 length
m1 = 1; %link 1 mass
m2 = 1; %link 2 mass
c1 = 1; % damping of link 1
c2 = 1; % damping of link 2
g=3.7;%acceleration due to gravity m/s^2 on mars

% equilibrium point
x_0 = [-pi/2,0,0,0]';%x_0=[q1_0,q1dot_0,q2_0,q2dot_0] initial conditions for the robot
x_0 = [ 0 0 0 0]';
e_0 = [0; 0];
% tau_0=[T1,T2]'; %initial torque done in constants2

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %Declare all your variables here, prefix with my_ 
% %Feel Free to add to or remove these constants
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
my_time = 0;
% my_angle_vector = [0 0]';
% my_state_estimate_vector = [0 0 0 0]';
my_delta_t = 0.001;
my_traj = 1;
my_traj_threshold = 0.01;
% my_some_variable_a=0;
% my_some_variable_b=0;
% my_tspan = 0:0.001:10; % set time interval
% my_options = odeset('RelTol',1e-5, 'AbsTol', 1e-6);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TRACJECTORY 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% y_ref = [[-pi/2+0.1; 0], [-pi/4; 0], [-pi/2; 0]];
% y_ref = [[0;0;0; 0], [-pi/4; 0], [-pi/2; 0]];
y_ref = [-pi/2;0;0;0]