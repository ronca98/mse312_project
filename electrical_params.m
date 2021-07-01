clear all 
close all
clc

%% input voltage, PWM and H-Bridge
command_voltage = 8.875; % V % 37.5% duty cycle for 60V output
pwm_freq = 4000; % Hz
Vm_min = 7; %V
Vm_max = 12; %V
max_output_voltage = 160; %V
voltage_reverse = 1; %V
output_on_resistance = 0.05; % ohms
if command_voltage < 0
    voltage_reverse = 1.5;
    command_voltage = abs(command_voltage);
end

%% DC motor
% Electrical 
% Option 1: model param: by stall torque and no load speed
armature_inductance = 0.0079; % H
no_load_speed = 1400; % rpm
stall_torque = 12; % N*m

% Mechanical 
rotor_inertia = 0.0057; % kg*m^2
rotor_damping = 0; % N*m/s(rad/s)
gear_ratio = 1;

%% run simulation
model = sim("electrical_model.slx", 1);
time_vector = model.speed_rpm.Time;
position_vector = model.position_deg.Data;
speed_vector = model.speed_rpm.Data;

data_array = cat(2, time_vector, position_vector, speed_vector);
data_array = data_array(data_array(:, 2) > 70, :);
launch_angle = data_array(1, 2);
launch_speed = data_array(1, 3);

text = ["Launch angle: ", launch_angle, "launch_speed: ", launch_speed];
disp(text)



