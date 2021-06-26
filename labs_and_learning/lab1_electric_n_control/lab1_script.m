clear all 
close all
clc

%% input voltage and PWM
pwm_ref_voltage = 2.5; % V
pwm_freq = 4000; % Hz


%% DC motor
% Electrical 
% Option 1: model param: by rated load and speed
armature_inductance = 0.01; % H
no_load_speed = 4000; % rpm
rated_speed = 2500; % rpm
rated_load_power = 10; %W
rated_dc_supply_voltage = 12; %V

% Mechanical 
rotor_inertia = 2000; % g*cm^2
rotor_damping = 1e-06; % N*m (rad/s)

