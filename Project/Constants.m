%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %You NEED these constants
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
l1 = 0.15; %link 1 length
l2 = 0.15; %link 2 length
m1 = l1*2.5; %link 1 mass
m2 = l2*2.5; %link 2 mass
c1 = 4; % damping of link 1
c2 = 4; % damping of link 2
g=3.7;%acceleration due to gravity m/s^2 on mars
% Starting point x_0=[q1_0,q1dot_0,q2_0,q2dot_0] initial conditions for the robot
x_0 = [ 0.3888 0 1.4367 0]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %Declare all your variables here, prefix with my_ 
% %Feel Free to add to or remove these constants
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
my_time = 0;
params = [m1, m2, l1, l2, c1, c2];
% my_angle_vector = [0 0]';
my_state_estimate_vector = x_0;
my_error = [0; 0];
% delta_x_hat0 = x_0; % - x_op %if you wanted to make your starting position ...
%and the linearization at different points
% delta_x_hat0 = x_0 + [1, 0, 1 , 0]';
my_delta_t = 0.001;
my_traj = 1;
my_traj_threshold = 0.01;
my_meas_noise_mean = 0;
my_meas_standard_devation = 1/3*pi/180;

% my_tspan = 0:0.001:10; % set time interval
% my_options = odeset('RelTol',1e-5, 'AbsTol', 1e-6);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TRACJECTORY 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% y_ref = [[-pi/2+0.1; 0; 0; 0], [-pi/4; 0], [-pi/2; 0]];
% y_ref = [[0;0;0; 0], [-pi/4; 0], [-pi/2; 0]];
y_ref = [0.3888 1.4367]';
% y_ref = [-pi/2;0;0;0];
y_ref = [0.4754 0.6200]';





