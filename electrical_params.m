clear all 
close all
clc

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
model = sim("electrical_model.slx", 0.1);
time_vector = model.speed_rpm.Time;

current_vector = model.current.Data;
power_vector = model.power.Data;
voltage_vector = model.voltage.Data;

position_vector = model.position_deg.Data;
speed_vector = model.speed_rpm.Data;

%% Calculations at swing angle
swing_angle = 75;
data_array = cat(2, ...
                 power_vector, current_vector, voltage_vector, ...
                 time_vector, position_vector, speed_vector);

peak_current = data_array(1, 2);

            
data_array = data_array(data_array(:, 5) > swing_angle, :);

launch_power = data_array(1, 1);
launch_voltage = data_array(1, 3);

launch_time = data_array(1, 4);
launch_angle = data_array(1, 5);
launch_speed = data_array(1, 6);

electrical_info = ["Peak Current: ", peak_current, ... 
                   "Power: ", launch_power, ...
                   "Voltage: ", launch_voltage];
disp(electrical_info)
               
mechanical_info = ["Launch time:", launch_time, ...
                   "Launch angle: ", launch_angle, ...
                   "Launch_speed: ", launch_speed];          
disp(mechanical_info)



