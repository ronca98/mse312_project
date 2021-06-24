close all
clc
clear all

%% parameters from class Calculations used for simulink, script
calc = Calculations;
m_arm = calc.MassArm;
cg_ball = calc.CGBall;
r_ball = calc.RadiusBall;

%% specify how much to swing the arm and rest position
max_torque = -2.7;
arm_swing_angle = -45-20; %degrees (rotating clockwise, maximum start at 180 degrees) 
arm_start_angle = 200; 

%% gear sizes radius % (cm)
base_gear = 0.9525;
follower_gear = 3.81;
gear_ratio = 4;

%% start positions at ball launch from origin, used for simulink, script
rotation_pivot_height = 4.5; %(cm)
z_distance_arm = 6.5; %cm

calc.X0 = cg_ball*cosd(arm_start_angle + arm_swing_angle) + (-follower_gear/100); % initial x position (of ball)(m)
calc.Y0 = cg_ball*sind(arm_start_angle + arm_swing_angle) + ((rotation_pivot_height+1.05+r_ball)/100) ; % initial y position (of ball)(m)

%% calculate required launch velocity from torque and starting and swing angle
v_x_y_launch = calc.launch_x_y_velocity(max_torque, arm_swing_angle, arm_start_angle, gear_ratio);

vx_launch = v_x_y_launch(1);
vy_launch = v_x_y_launch(2);

%% calculate landing x distance and time of landing of ball
x_landing_time = calc.landing_distance_and_time(vx_launch , vy_launch);

x_landing = x_landing_time(1);
t_landing = x_landing_time(2);

sim("simscape_main.slx", 2.5)

%% create vectors from calculated data:
d_vectors = calc.x_y_d_vectors(vx_launch, vy_launch, t_landing);
x_vector = d_vectors(:, 1);
y_vector = d_vectors(:, 2);

%% obtain simscape data
% Need to offset x and y data as they start at zero due to a different
% joint keeping track of motion and thus another reference frame.
x_data = position_x.Data + calc.X0;
y_data = position_y.Data + calc.Y0;
y_data = y_data(y_data>0);
x_data = x_data(1:length(y_data));
x_data_landing = x_data(end);
y_data_max = max(y_data);
t_data = position_x.Time;
t_data = t_data(1:length(y_data));
t_data_landing = t_data(end);

%% plot calculation vs simulation

plot(x_vector, y_vector, 'o')
hold on 
plot(x_data, y_data)
hold off






