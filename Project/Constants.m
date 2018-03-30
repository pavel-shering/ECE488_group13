pull = 1
wait = 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %You NEED these constants
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% alum
% l1 = 0.441176; %link 1 length
% l2 = 0.441176; %link 2 length
% m1 = l1*0.85; %link 1 mass
% m2 = l2*0.85; %link 2 mass
% % c1 = 6 + rand(1)*4; % damping of link 1
% % c2 = 6 + rand(1)*4; % damping of link 2
% c1 = 8; % damping of link 1
% c2 = 8; % damping of link 2
% titanium 
l1 = 0.2678; %link 1 length
l2 = 0.2678; %link 2 length
% m1 = (1.4*l1 - 0.1*1.4*l1) + rand(1)*(0.2*1.4*l1); %link 1 mass
% m2 = 0.75 - m1; %link 2 mass
% c1 = 4 + rand(1)*4; % damping of link 1
% c2 = 4 + rand(1)*4; % damping of link 2
m1 = 1.4*l1; %l1 mass
m2 = 1.4*l2;
c1 = 6; % damping of link 1
c2 = 6; % damping of link 2
% steel
% l1 = 0.15; %link 1 length
% l2 = 0.15; %link 2 length
% m1 = l1*2.5; %link 1 mass
% m2 = l2*2.5; %link 2 mass
% c1 = 4; % damping of link 1
% c2 = 4; % damping of link 2

% gold and aluminum

% steel and aluminum 

% titanium and aluminum


g=3.7;%acceleration due to gravity m/s^2 on mars
% Starting point x_0=[q1_0,q1dot_0,q2_0,q2dot_0] initial conditions for the robot


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %Declare all your variables here, prefix with my_ 
% %Feel Free to add to or remove these constants
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
my_time = 0;
my_params = [m1, m2, l1, l2, c1, c2];
% my_angle_vector = [0 0]';

% my_state_estimate_vector = [0 0 0 0]';
% delta_x_hat0 = x_0; % - x_op %if you wanted to make your starting position ...
%and the linearization at different points
% delta_x_hat0 = x_0 + [1, 0, 1 , 0]';
my_error = [0; 0];
my_delta_t = 0.001;

if pull 
    my_traj_count = 1;
else 
    my_traj_count = 2;
end
my_traj_threshold = 0.05;
my_tspan = 0:0.001:10; % set time interval

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TRACJECTORY 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
target_A = [0.10 0.20];
target_B = [0.20 0.20];
target_C = [0.20 0.10];
target_D = [0.10 0.10];

% milestones = [ A B C D A ]
milestones = [ target_A; 
               target_B;
               target_C;
               target_D;
               target_A];
           
my_inbw_points = 2 + 2;
my_traj_xy = [];

% This variable is used to check if within 4mm of milestone
my_milestone_ctr = 2;

% Global boolean for if we are waiting for 0.5s
recordedMilestoneStartTime = 0;
% Global variabble for waiting for 0.5s
milestoneStartTime = 0;

% delete adjacent duplicates
for k = 1:length(milestones)-1
    xvalues = linspace(milestones(k,1), milestones(k+1,1), my_inbw_points);
    yvalues = linspace(milestones(k,2), milestones(k+1,2), my_inbw_points);
    for i = 1:my_inbw_points
        temp(i, :) = [xvalues(i) yvalues(i)];
    end
    
    my_traj_xy = [my_traj_xy; temp];
end

c2_ik = (my_traj_xy(:,1).^2 + my_traj_xy(:,2).^2 - l1^2 - l2^2)/(2*l1*l2);
s2_ik = sqrt(1 - c2_ik.^2);
THETA2D = atan2(s2_ik,c2_ik);

k1_ik = l1 + l2*c2_ik;
k2_ik = l2*s2_ik;
THETA1D = atan2(my_traj_xy(:,2),my_traj_xy(:,1)) - atan2(k2_ik,k1_ik); % theta1 is deduced

% forward kinematics test
% X_pred = l1 * cos(THETA1D) + l2 * cos(THETA1D + THETA2D); % compute x coordinates
% Y_pred = l1 * sin(THETA1D) + l2 * sin(THETA1D + THETA2D);

my_traj_angles = [THETA1D THETA2D]
x_0 = [my_traj_angles(1,1) 0 my_traj_angles(1,2) 0]';

if pull 
    my_delta_xhat = [0 0 0 0]';
else
    my_delta_xhat = [0 0 0 0]';
end

