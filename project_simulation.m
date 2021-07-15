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

%% Inertia calculations
J_shaft = 4.05*0.0001*(1/1000);
% inertia for input
J_base_gear = 0.46*0.0001*(1/1000);
J_1 = J_shaft + J_base_gear;

% inertia for output
J_arm = (14532.12*0.0001*(1/1000)); % kg*m^2
J_follower_gear = 325.50*0.0001*(1/1000);
J_ball = ((2/5)*m_ball*r_ball^2) + (m_ball*cg_ball^2);
J_2 = J_arm + J_follower_gear + J_shaft + J_ball;
% Total inertia experienced at the input
J_total =  J_1 + (J_2/(gear_ratio)^2); 

%% Load in parameters from electrical component
electrical_and_controls;

%% specify how much to swing the arm and rest position
input_torque = 0.3;
arm_swing_angle = -45; %degrees (rotating clockwise, maximum start at 180 degrees) 
arm_start_angle = 160; 

%% start positions at ball launch from origin, used for simulink, script
rotation_pivot_height = 4.09; %(cm)
z_distance_arm = 6.5; %cm

x0 = cg_ball*cosd(arm_start_angle + arm_swing_angle); % initial x position (of ball)(m)
x0_rest = cg_ball*cosd(arm_start_angle);
y0 = cg_ball*sind(arm_start_angle + arm_swing_angle) ... 
          + ((rotation_pivot_height+1.05+r_ball+center_distance)/100) ; % initial y position (of ball)(m)
y0_rest = cg_ball*sind(arm_start_angle) ... 
          + ((rotation_pivot_height+1.05+r_ball+center_distance)/100);

%% calculate required launch velocity from torque and starting and swing angle
v_x_y_launch = calc.launch_x_y_velocity(input_torque, ...
                                        arm_swing_angle, arm_start_angle, ...
                                        J_total, gear_ratio, ...
                                        cg_ball);

vx_launch = v_x_y_launch(1);
vy_launch = v_x_y_launch(2);

%% calculate landing x distance and time of landing of ball
x_landing_time = calc.landing_distance_and_time(vx_launch , vy_launch, ...
                                                x0, y0);

x_landing = x_landing_time(1);
t_landing = x_landing_time(2);

sim("simscape_main.slx", 2.5)

%% create vectors from calculated data:
d_vectors = calc.x_y_d_vectors(vx_launch, vy_launch, ... 
                               t_landing, ...
                               x0, y0);
x_vector = d_vectors(:, 1);
y_vector = d_vectors(:, 2);
y_max = max(y_vector);

%% obtain simscape data and perform calculations
% Need to offset x and y data as they start at zero due to a different
% joint keeping track of motion and thus another reference frame.
t_data = position_x.Time;
x_data = position_x.Data+x0_rest;
y_data = position_y.Data+y0_rest;
impact_force_data = impact_force.Data;

ball_data = cat(2, t_data, x_data, impact_force_data);
ball_data = ball_data(ball_data(:, 3) > 1000, :);

t_data_land = ball_data(1,1);
x_data_land = ball_data(1,2);
y_data_max = max(y_data);

%% display information for user
disp("project_simulation.m:")
calc_info = ["t_land (s): ", t_landing, ...
             "x_land (m): ", x_landing, ...
             "y_max (m): ", y_max];
disp(calc_info)
ball_info = ["t_data_land (s): ", t_data_land, ...
             "x_data_land (m): ", x_data_land, ...
             "y_data_max (m): ", y_data_max];
disp(ball_info)

%% plot calculation vs simulation
plot(x_vector, y_vector, 'o')
hold on 
plot(x_data, y_data)
ylabel("position y (m)")
yyaxis right
plot(x_data, impact_force_data)
hold off
xlabel("position x (m)")







