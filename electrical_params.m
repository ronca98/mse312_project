close all
clc
check = exist("mean_driving_torque",  "var");
if check == 1
    clear all 
end

%% input voltage, PWM and H-Bridge
desired_duty_cycle = 1; 
pwm_freq = 4000; % Hz
Vm_min = 10; %V
Vm_max = 45; %V
command_voltage = (Vm_min + (Vm_max-Vm_min))*desired_duty_cycle; % V
output_on_resistance = 0.55; % ohms

%% DC motor
% Electrical 
% Option 1: model param: by stall torque and no load speed
rated_dc_supply_voltage = 12; %V
armature_inductance = 0.00234; % H
no_load_speed = 720; % rpm
stall_torque = 0.3; % N*m

% Mechanical 
rotor_inertia = 1.6e-06 + 2.4933e-04; % kg*m^2
rotor_damping = 1.4e-06; % N*m/s(rad/s)
% gear ratio for external gears 
gear_ratio = 4.8;
%% run simulation
model = sim("electrical_model.slx", 1);
time_vector = model.speed_rpm.Time;

current_vector = model.current.Data;
power_vector = model.power.Data;
voltage_vector = model.voltage.Data;

position_vector = model.position_deg.Data;
speed_vector = model.speed_rpm.Data;
driving_torque_vector = model.driving_torque.Data;

%% calculations for tables
swing_angle = 75;
data_array = cat(2, ...
                 power_vector, current_vector, voltage_vector, ...
                 time_vector, position_vector, speed_vector, driving_torque_vector);
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
disp("electrical_params.m:")
electrical_info = ["Peak Current: ", peak_current, ... 
                   "Power: ", launch_power, ...
                   "Voltage: ", launch_voltage];
disp(electrical_info)

rotation_direction_text = "CCW";
if command_voltage < 0
    rotation_direction_text = "CW";
end
               
mechanical_info = ["Launch time:", launch_time, ...
                   "Launch angle: ", launch_angle, ...
                   "Launch_speed: ", launch_speed];          
disp(mechanical_info)

% torque_info = ["Peak Driving Torque", peak_driving_torque, ...
%                "Launch Driving Torque",  launch_driving_torque, ...
%                rotation_direction_text, ...
%                "Mean Driving Torque", mean_driving_torque];
% disp(torque_info)



