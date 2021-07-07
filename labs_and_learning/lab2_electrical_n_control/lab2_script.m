clear all 
close all
clc

%% eletromech system transfer function, controller setup

% parameters:
n = 6.3; % gear ratio
eta = 0.95; % gear efficiency
% J_rotor = 1.6e-06; % rotor inertia, kg*m^2
% J_load = (14532.12*0.0001*(1/1000)); % load inertia, kg*m^2
% J = n^2 * J_rotor + J_load; % total inertia, kg*m^2
J = 0.083; % total inertia, kg*m^2
k_m = 2.18e-02; % torque constant, N-m/A
% B = 1.4e-06; % viscous mech damping, N-m-s
B = 10^(-3); % viscous mech damping, N-m-s
R_m = 4.33; % equivalent motor resistance, Ohm
L_m = 2.34e-03; % equivalent motor inductance, H
sample_bw = 64; % sampling bandwidth, Hz

% transfer function:
numer = [J B];
denom = [(L_m*J) (R_m*J + B*L_m) (R_m*B + eta*n^2*k_m*2)];
motor_tf = tf(numer, denom);
approx_motor_tf = tf(1,[L_m R_m]);  % approximated by cancelling dominant pole and zero

% obtain zeros and poles:
z = zero(motor_tf);
p = pole(motor_tf);
z_app = zero(approx_motor_tf);
p_app = pole(approx_motor_tf);

% PI controller design
a = 6; % mutiplier
k_p = (2 * pi * sample_bw * a) * L_m;
k_i = k_p * R_m / L_m;
control_tf = tf([k_p k_i], [1 0]);

% reference or desired current
i_ref = -0.03; % A

% verify controller with 5-sec unit step response:
% sys = feedback(control_tf * approx_motor_tf, 1);
% step(sys, 5);

%% PWM and H-Bridge parameters
pwm_freq = 4000; % Hz
max_voltage_input_hbridge = 5;

% duty_cycle = 0.5; % between 0 and 1
% Vm_min = 10;
% Vm_max = 45;
% command_voltage = -(((Vm_max - Vm_min) * duty_cycle) + Vm_min); % V
% voltage_reverse = 1;
% if command_voltage < 0
%     voltage_reverse = 2.6;
%     command_voltage = abs(command_voltage);
% end

%% DC motor
% Electrical 
% Option 2: model param by equivalent circuit parameters
rated_dc_supply_voltage = 12; %V
armature_resistance = 4.33; % Ohm
armature_inductance = 0.00234; % H
no_load_speed = 720; % rpm
back_emf_constant = 2.18e-2; % V/(rad/s)

% Mechanical 
rotor_inertia = 1.6e-06; % kg*m^2
rotor_damping = 1.4e-06; % N*m/(rad/s)
gear_ratio = 6.3;

%% run simulation
sim('DC_motor_model.slx', 10)

%% show plots
figure;
subplot(2, 2, 1); plot(ans.power, 'LineWidth', 2); grid on;
subplot(2, 2, 2); plot(ans.current, 'LineWidth', 2); grid on;
subplot(2, 2, 3); plot(ans.voltage, 'LineWidth', 2); grid on;
subplot(2, 2, 4); plot(ans.rpm, 'LineWidth', 2); grid on;