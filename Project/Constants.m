%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %You NEED these constants
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% titanium 
l1 = 0.27; %link 1 length
% noise testing
% m1 = (1.4*l1 - 0.1*1.4*l1) + rand(1)*(0.2*1.4*l1); %link 1 mass
% m2 = 0.75 - m1; %link 2 mass
% l2 = m2/1.4; %link 2 length
% c1 = 4 + rand(1)*4; % damping of link 1
% c2 = 4 + rand(1)*4; % damping of link 2
% actual params without variations
m1 = 1.4*l1; %l1 mass
m2 = 0.75-m1;
l2 = m2/1.4; %link 2 length
c1 = 6; % damping of link 1
c2 = 6; % damping of link 2
g=3.7;%acceleration due to gravity m/s^2 on mars

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
my_time = 0;
my_params = [m1, m2, l1, l2, c1, c2];
my_delta_xhat = [0 0 0 0]';
my_error = [0; 0];
my_delta_t = 0.001;
my_energy = 0;
my_pull = 1;
my_wait = 1;
my_sd = (1/3)*(pi / 180);

if my_pull 
    my_traj_count = 1;
else 
    my_traj_count = 2;
end
my_traj_threshold = 0.05;
my_tspan = 0:0.001:10; % set time interval

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TRACJECTORY 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
           
my_inbw_points = 2 + 10;
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

my_traj_angles = [THETA1D THETA2D];
x_0 = [my_traj_angles(1,1) 0 my_traj_angles(1,2) 0]';
