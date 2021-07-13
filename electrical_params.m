close all
clc
check = exist("mean_driving_torque",  "var");
if check == 1
    clear all 
end

%% input voltage, PWM and H-Bridge
desired_duty_cycle = 1; 
pwm_freq = 4000; % Hz
Vm_min = 0; %V
Vm_max = 12; %V
command_voltage = (Vm_min + (Vm_max-Vm_min))*desired_duty_cycle; % V
output_on_resistance = 0.55; % ohms

%% DC motor
% Electrical 
% Option 1: model param: by stall torque and no load speed
rated_dc_supply_voltage = 12; %V
R_m = 4.33; % equivalent motor resistance, Ohm
L_m = 0.00234; % armature inductance H
no_load_speed = 720; % rpm
stall_torque = 0.3; % N*m
torque_const = 2.18e-02; % N*m/A
back_emf_const = 2.18e-02;

% Mechanical 
system_inertia = 2.4933e-04; % kg*m^2
rotor_inertia = 6.3^2*1.6e-06; % + system_inertia; % kg*m^2
rotor_damping = 1.4e-06; % N*m/s(rad/s)
% gear ratio for external gears 
gear_ratio_int = 6.3;
gear_ratio_ext = 1;
train_ratio = gear_ratio_int*gear_ratio_ext;
 
%% Controllers parameters
sample_freq = 128;
sample_bw_rad = 2*pi*sample_freq;
% Current
k_p = (sample_bw_rad*L_m/10);
k_i = (sample_bw_rad*R_m/10);
% Speed
k_p_w = ((sample_bw_rad*rotor_inertia)/100)*25;
k_i_w = ((sample_bw_rad*rotor_damping)/100);
% Position
k_p_p = 35;

% reference signal
% i_ref = 0.6; % A
w_ref = 65; % omega
pos_d = 45; % degrees

%% generate input signal
t_final = 1.2;
period = (1/sample_freq)*0.1;
% parabolic
time_phase_one = 0:period:0.02;
y_phase_one =  45000*time_phase_one.^2;
% ramp
time_phase_two = (0.02+period):period:0.05;
y_phase_two = ((pos_d-y_phase_one(end))/(0.03-period))*time_phase_two;
% quintic
time_pts_phase_three = [(0.05+period), 0.2, 0.4, 0.6, 0.8, 1, t_final];
y_pts_phase_three = [pos_d, 30, 15, 10, 4, 2, 0];
time_phase_three = (0.05+period):period:t_final;
quintic_polynomial = polyfit(time_pts_phase_three, y_pts_phase_three, 5);
y_phase_three = polyval(quintic_polynomial, time_phase_three);
% combine into array
input_time = cat(1, time_phase_one', time_phase_two', time_phase_three');
input_y = cat(1, y_phase_one', y_phase_two', y_phase_three');
pos_ref = [input_time, input_y];

%% run simulation
model = sim("electrical_model.slx", t_final);

%% Load data from Simulink
time_vector = model.speed_rpm.Time;

current_vector = model.current.Data;
power_vector = model.power.Data;
voltage_vector = model.voltage.Data;

position_ref_vector = model.position_deg_ref.Data;
position_vector = model.position_deg.Data;
speed_vector = model.speed_rpm.Data;
driving_torque_vector = model.driving_torque.Data;

%% Plot Data
% Various Electrical Plots
figure;
subplot(2, 2, 1); plot(time_vector, power_vector, 'LineWidth', 2); grid on;
title("Power (W)")
subplot(2, 2, 2); plot(time_vector, current_vector, 'LineWidth', 2); grid on;
title("Current (A)")
subplot(2, 2, 3); plot(time_vector, voltage_vector, 'LineWidth', 2); grid on;
title("Voltage (V)")
subplot(2, 2, 4); plot(time_vector, speed_vector, 'LineWidth', 2); grid on;
title("Speed (RPM)")

% Reference vs Actual Position
figure
plot(time_vector, position_ref_vector);
hold on
plot(time_vector, position_vector);
hold off
legend("position reference", "position actual");
ylabel("\theta ")
xlabel("time (s)")

% Plot Errors
figure;
subplot(2, 2, 1); plot(time_vector, model.error_position.Data, 'LineWidth', 2); grid on;
title("Position Error")
subplot(2, 2, 2); plot(time_vector, model.error_speed.Data, 'LineWidth', 2); grid on;
title("Speed Error")
subplot(2, 2, 3); plot(time_vector, model.error_current.Data, 'LineWidth', 2); grid on;
title("Current Error")

%% Obtain calculations from Data
data_array = cat(2, ...
                 power_vector, current_vector, voltage_vector, ...
                 time_vector, position_vector, speed_vector, driving_torque_vector);            
swing_angle = 75;
filtered_current_vector = data_array(:, 2);
[~, idx] = max(abs(filtered_current_vector));
peak_current = filtered_current_vector(idx);

filtered_driving_torque_vector = data_array(:, 7);
[~, idx] = max(abs(filtered_driving_torque_vector));
peak_driving_torque = filtered_driving_torque_vector(idx);

data_array = data_array(data_array(:, 5) < swing_angle, :);

launch_power = data_array(end, 1);
launch_voltage = data_array(end, 3);

launch_time = data_array(end, 4);
launch_angle = data_array(end, 5);
launch_speed = data_array(end, 6);
launch_driving_torque = data_array(end, 7);
mean_driving_torque = mean(data_array(:, 7));

%% display information for user
% disp("electrical_params.m:")
% electrical_info = ["Peak Current: ", peak_current, ... 
%                    "Power: ", launch_power, ...
%                    "Voltage: ", launch_voltage];
% disp(electrical_info)
% 
% rotation_direction_text = "CCW";
% if command_voltage < 0
%     rotation_direction_text = "CW";
% end
               
% mechanical_info = ["Launch time:", launch_time, ...
%                    "Launch angle: ", launch_angle, ...
%                    "Launch_speed: ", launch_speed];          
% disp(mechanical_info)

% torque_info = ["Peak Driving Torque", peak_driving_torque, ...
%                "Launch Driving Torque",  launch_driving_torque, ...
%                rotation_direction_text, ...
%                "Mean Driving Torque", mean_driving_torque];
% disp(torque_info)



