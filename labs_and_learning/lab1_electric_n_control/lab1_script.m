clear all 
close all
clc

%% input voltage, PWM and H-Bridge
command_voltage = 27.5; % V
pwm_freq = 4000; % Hz
Vm_min = 10;
Vm_max = 45;
max_voltage_input_hbridge = 5;
voltage_reverse = 1; 
if command_voltage < 0
    voltage_reverse = 2.6;
    command_voltage = abs(command_voltage);
end

%% DC motor
% Electrical 
% Option 1: model param: by rated load and speed
rated_dc_supply_voltage = 12; %V
armature_inductance = 0.00234; % H
no_load_speed = 720; % rpm
stall_torque = 3.0e-01;
rated_speed = 650; % rpm
rated_load_power = 3.5; %W

% Mechanical 
rotor_inertia = 1.6e-06; % kg*m^2
rotor_damping = 1.4e-06; % N*m/s(rad/s)
gear_ratio = 5;

%% run simulation
sim('DC_motor_model.slx', 10)

