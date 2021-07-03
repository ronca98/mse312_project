close all
clc
check = exist("mean_driving_torque",  "var");
if check == 1
    clear all 
end

%% input voltage, PWM and H-Bridge
desired_duty_cycle = 1; 
pwm_freq = 4000; % Hz
Vm_min = 7; %V
Vm_max = 58; %V
command_voltage = Vm_min + (Vm_max-Vm_min)*desired_duty_cycle; % V
voltage_reverse = 1; %V
if command_voltage < 0
    voltage_reverse = 2.6;
    command_voltage = abs(command_voltage);
end
output_on_resistance = 0.55; % ohms

%% DC motor
% Electrical 
% Option 1: model param: by stall torque and no load speed
rated_dc_supply_voltage = 45; %V
armature_inductance = 0; % H
no_load_speed = 5800; % rpm
stall_torque = 13.13; % N*m

% Mechanical 
rotor_inertia = 2.31e-04; % kg*m^2
rotor_damping = 0; % N*m/s(rad/s)
gear_ratio = 5;

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

peak_current = data_array(1, 2);
peak_driving_torque = max(data_array(:, 7));

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
               
mechanical_info = ["Launch time:", launch_time, ...
                   "Launch angle: ", launch_angle, ...
                   "Launch_speed: ", launch_speed];          
disp(mechanical_info)

torque_info = ["Peak Driving Torque", peak_driving_torque, ...
               "Launch Driving Torque",  launch_driving_torque, ...
               "Mean Driving Torque", mean_driving_torque];
disp(torque_info)



