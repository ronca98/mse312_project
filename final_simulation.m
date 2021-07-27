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

%% specify how much to swing the arm and rest position
arm_swing_angle = -65; %degrees (rotating clockwise, maximum start at 180 degrees) 
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

%% input voltage, PWM and H-Bridge
desired_duty_cycle = 1; 
pwm_freq = 4000; % Hz
Vm_min = 0; %V
Vm_max = 24; %V
command_voltage = (Vm_min + (Vm_max-Vm_min))*desired_duty_cycle; % V
output_on_resistance = 2; % ohms

%% DC motor
% Option 1: model param: by stall torque and no load speed
% Electrical 
rated_dc_supply_voltage = 24; %V
R_m = 0.7; % equivalent motor resistance, Ohm
L_m = 0.00105; % armature inductance H
no_load_speed = 3700; % rpm
stall_torque = 2.0; % N*m
torque_const = 0.059; % N*m/A

% Mechanical 
gear_ratio_int = 0.6;
gear_ratio_ext = 4.8;
system_inertia = 2.4933e-04; % kg*m^2
rotor_inertia = gear_ratio_int^2*3.5e-05; %+ system_inertia; % kg*m^2
rotor_damping = 1.50e-06; % N*m/s(rad/s)
% Gear Ratios
train_ratio = gear_ratio_int*gear_ratio_ext;
 
%% Controllers parameters
sample_freq = 128;
sample_bw_rad = 2*pi*sample_freq;
% Current
k_p = (sample_bw_rad*L_m/10)*6;
k_i = (sample_bw_rad*R_m/10)*4;
% Speed
k_p_w = ((sample_bw_rad*rotor_inertia)/100)*3600;
k_i_w = ((sample_bw_rad*rotor_damping)/100);
% Position
k_p_p = 38;
k_i_p = 1;
k_d_p = 0.001;
% reference signal
pos_d = -arm_swing_angle; % degrees
speed_ramp_t = 0.1;
w_d = (pos_d/speed_ramp_t)*(pi/180)*9.55; % rpm
t_final = 4.5;
period = (1/sample_freq)*0.01;

%% generate input signal for speed
% phase one
time_phase_one = 0:period:speed_ramp_t;
y_phase_one = (w_d/speed_ramp_t)*(time_phase_one);
% phase two
time_phase_two = (speed_ramp_t+period):period:t_final;
y_phase_two = y_phase_one(end)*ones(1, length(time_phase_two));
% create reference speed signal
input_time = cat(1, time_phase_one', time_phase_two');
input_y = cat(1, y_phase_one', y_phase_two');
input_y_integral = cumtrapz(input_time, input_y*0.10472);
w_ref = [input_time, input_y];
% determine the time when phase 2 ends 
position_array = cat(2, input_time, input_y_integral*(180/pi));
phase_one_and_two = position_array(position_array(:, 2) < pos_d, :);
t_ref_launch = phase_one_and_two(end, 1);

%% generate input signal for position
n_pts = 7;
time_pts_phase_three = t_ref_launch + linspace(0, t_final, 7);
y_pts_phase_three = linspace(pos_d, 0, 7);
y_pts_phase_three(2) = y_pts_phase_three(2)*0.7;
y_pts_phase_three(3) = y_pts_phase_three(2)*0.6;
y_pts_phase_three(4) = y_pts_phase_three(4)*0.4;
y_pts_phase_three(5) = y_pts_phase_three(5)*0.2;
y_pts_phase_three(6) = y_pts_phase_three(6)*0.1;
time_phase_three = t_ref_launch + (0:period:t_final);
quintic_polynomial = polyfit(time_pts_phase_three, y_pts_phase_three, 5);
y_phase_three = polyval(quintic_polynomial, time_phase_three);
phase_three = cat(2, time_phase_three', y_phase_three');
pos_ref = cat(1, phase_one_and_two, phase_three);

%% run simulation
model = sim("final_simscape.slx", t_final);

%% Load data from Simulink
time_vector = model.speed_rpm.Time;

current_vector = model.current.Data;
power_vector = model.power.Data;
voltage_vector = model.voltage.Data;

speed_ref_vector = model.speed_rpm_ref.Data;
position_ref_vector = model.position_deg_ref.Data;
position_vector = model.position_deg.Data;
speed_vector = model.speed_rpm.Data;
driving_torque_vector = model.driving_torque.Data;

%% Plot Data
% Various Electrical Plots
figure;
subplot(2, 2, 1); plot(time_vector, power_vector); grid on;
title("Power (W)")
subplot(2, 2, 2); plot(time_vector, current_vector); grid on;
title("Current (A)")
subplot(2, 2, 3); plot(time_vector, voltage_vector); grid on;
title("Voltage (V)")
subplot(2, 2, 4); plot(time_vector, speed_vector); grid on;
title("Speed (RPM)")

% Reference vs Actual Speed
figure
plot(time_vector, speed_ref_vector);
hold on
plot(time_vector, speed_vector);
hold off
grid on
legend("speed reference", "speed actual");

title("\omega (RPM)")
xlabel("time (s)")

% Reference vs Actual Position
figure
plot(time_vector, position_ref_vector);
hold on
plot(time_vector, position_vector);
hold off
grid on
legend("position reference", "position actual")
title("\theta (degrees)")
xlabel("time (s)")

% Plot Errors
figure;
subplot(2, 2, 1); plot(time_vector, model.error_position.Data); grid on;
title("Position Error")
subplot(2, 2, 2); plot(time_vector, model.error_speed.Data); grid on;
title("Speed Error")
subplot(2, 2, 3); plot(time_vector, model.error_current.Data); grid on;
title("Current Error")

%% Obtain calculations from Data
data_array = cat(2, ...
                 power_vector, current_vector, voltage_vector, ...
                 time_vector, position_vector, speed_vector, driving_torque_vector);            
swing_angle = pos_d;
filtered_current_vector = data_array(:, 2);
[~, idx] = max(abs(filtered_current_vector));
peak_current = filtered_current_vector(idx);

[~, idx] = max(position_vector);

launch_power = data_array(idx, 1);
launch_voltage = data_array(idx, 3);

launch_time = data_array(idx, 4);
launch_angle = data_array(idx, 5);
launch_speed = max(data_array(:, 6));

%% display information for user
disp("electrical_params.m:")
electrical_info = ["Peak Current: ", peak_current, ... 
                   "Power: ", launch_power, ...
                   "Voltage: ", launch_voltage];
disp(electrical_info)
              
mechanical_info = ["Launch time:", launch_time, ...
                   "Launch angle: ", launch_angle, ...
                   "Launch_speed: ", launch_speed];          
disp(mechanical_info)







