close all
clc
clear all

%% parameters from class Calculations used for simulink, script
calc = Calculations;
m_ball = 0.145; % (kg)
cg_ball = 0.17; % (m)% measured from rotation point
r_ball = 0.0315; % (m)

%% gear sizes radius % (cm)
base_gear = 0.8331;
follower_gear = 3.849;
gear_ratio = 4.8;
center_distance = base_gear+follower_gear;

% %% Inertia calculations
% J_shaft = 4.05*0.0001*(1/1000);
% % inertia for input
% J_base_gear = 0.446*0.0001*(1/1000);
% J_1 = J_shaft + J_base_gear;
% 
% % inertia for output
% J_arm = (14532.12*0.0001*(1/1000)); % kg*m^2
% J_follower_gear = 325.50*0.0001*(1/1000);
% J_ball = ((2/5)*m_ball*r_ball^2) + (m_ball*cg_ball^2);
% J_2 = J_arm + J_follower_gear + J_shaft + J_ball;
% % Total inertia experienced at the input
% J_total =  J_1 + (J_2/(gear_ratio)^2); 

%% specify how much to swing the arm and rest position
arm_swing_angle = -45; %degrees (rotating clockwise, maximum start at 180 degrees) 
arm_start_angle = 180; 

%% start positions at ball launch from origin, used for simulink, script
rotation_pivot_height = 4.09; %(cm)
z_distance_arm = 6.5; %cm

x0 = cg_ball*cosd(arm_start_angle + arm_swing_angle); % initial x position (of ball)(m)
x0_rest = cg_ball*cosd(arm_start_angle);
y0 = cg_ball*sind(arm_start_angle + arm_swing_angle) ... 
          + ((rotation_pivot_height+1.05+r_ball+center_distance)/100) ; % initial y position (of ball)(m)
y0_rest = cg_ball*sind(arm_start_angle) ... 
          + ((rotation_pivot_height+1.05+r_ball+center_distance)/100);

electrical_and_controls;
% sim("simscape_main.slx", 2.5)








