close all
clc
% clear all

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

%% use model to give launch angle for specified distance
polynomial_coeffs = readmatrix("curve_fit_model_0.2_to_0.9.csv");

if x_specified > 0.9
    polynomial_coeffs = readmatrix("curve_fit_model_0.9_to_1.5.csv");
end

%% specify how much to swing the arm and rest position
arm_swing_angle = polyval(polynomial_coeffs, x_specified); %degrees (rotating clockwise, maximum start at 180 degrees) 
if x_specified > 1.5
    arm_swing_angle = -80;
end
arm_start_angle = 204.8; 

%% start positions at ball launch from origin, used for simulink, script
rotation_input_height = 4.09; %(cm)
z_distance_arm = 3.75; %cm
z_distance_gears = z_distance_arm+1;
x_distance_origin = -30; %cm

x0 = cg_ball*cosd(arm_start_angle + arm_swing_angle) + (x_distance_origin/100); % initial x position (of ball)(m)
x0_rest = cg_ball*cosd(arm_start_angle) + (x_distance_origin/100) ;
y0 = cg_ball*sind(arm_start_angle + arm_swing_angle) ... 
          + (((rotation_input_height+1.05+center_distance)/100) + r_ball) ; % initial y position (of ball)(m)
y0_rest = cg_ball*sind(arm_start_angle) ... 
          + (((rotation_input_height+1.05+center_distance)/100) + r_ball);

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
k_p = (sample_bw_rad*L_m/10)*10;
k_i = (sample_bw_rad*R_m/10)*5;
% Speed
k_p_w = ((sample_bw_rad*rotor_inertia)/100)*2000;
k_i_w = ((sample_bw_rad*rotor_damping)/100)*3000;
% Position
k_p_p = 32;
k_i_p = 1;
k_d_p = 0.01;
% reference signal
pos_d = -arm_swing_angle; % degrees
speed_ramp_t = 0.055;
w_d = (pos_d/speed_ramp_t)*(pi/180)*9.55; % rpm
t_final = 3;
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
pos_d_offset = abs(180-arm_start_angle);
position_array = cat(2, input_time, (input_y_integral*(180/pi)) - pos_d_offset);

phase_one_and_two = position_array(position_array(:, 2) < pos_d - pos_d_offset, :);
t_ref_launch = phase_one_and_two(end, 1);

%% generate input signal for position
n_pts = 7;
time_pts_phase_three = t_ref_launch + linspace(0, t_final, 7);
y_pts_phase_three = linspace(phase_one_and_two(end), 0-pos_d_offset, 7);
y_pts_phase_three(2) = y_pts_phase_three(2)-y_pts_phase_three(2)*0.3;
y_pts_phase_three(3) = y_pts_phase_three(2)-y_pts_phase_three(2)*0.4;
y_pts_phase_three(4) = y_pts_phase_three(4)-y_pts_phase_three(2)*0.6;
y_pts_phase_three(5) = y_pts_phase_three(5)-y_pts_phase_three(2)*0.3;
y_pts_phase_three(6) = y_pts_phase_three(6)-y_pts_phase_three(2)*0.25;
time_phase_three = t_ref_launch + (0:period:t_final);
quintic_polynomial = polyfit(time_pts_phase_three, y_pts_phase_three, 5);
y_phase_three = polyval(quintic_polynomial, time_phase_three);

phase_three = cat(2, time_phase_three', y_phase_three');
pos_ref = cat(1, phase_one_and_two, phase_three);

%% run simulation
model = sim("final_simscape.slx", t_final+0.15);

%% Load data from Simulink
time_vector = model.speed_rpm.Time;
current_vector = model.current.Data;
power_vector = model.power.Data;
power_consumption_vector = model.power_consumption.Data;
voltage_vector = model.voltage.Data;
speed_ref_vector = model.speed_rpm_ref.Data;
position_ref_vector = model.position_deg_ref.Data;
position_vector = model.position_deg.Data;
input_torque_vector = model.input_torque.Data;
arm_acceleration_vector = model.arm_acceleration.Data;
speed_vector = model.speed_rpm.Data;
x_data = model.position_x.Data;
y_data = model.position_y.Data;
ball_velocity_vector = model.ball_velocity.Data;
ball_acceleration_vector = model.ball_acceleration.Data;
back_to_start_timer = model.timer.Data;
back_to_start_timer = round(back_to_start_timer, 4);
back_to_start_position = model.position_deg_final_timer.Data;
x_data_impact = model.position_x_impact.Data;
t_data_impact = model.position_x_timer.Data;

%% Plot Data
graph_results_folder = "graphs_folder/%gm_graph_results";
graph_results_folder = sprintf(graph_results_folder, x_specified);
mkdir(graph_results_folder);

% Ball x vs y Position
figure
plot(x_data, y_data)
hold on
ylabel("position y (m)")
hold off
xlabel("position x (m)")
title("Ball position x vs y")
graph1_file_name = "%gm_position_ball_x_vs_y.png";
graph1_file_name = sprintf(graph1_file_name, x_specified);
graph1_file_name = fullfile(graph_results_folder, graph1_file_name);
saveas(gcf, graph1_file_name);

% Ball velocity vs time 
figure
plot(time_vector, ball_velocity_vector);
ylabel("velocity (m/s)")
xlabel("time (s)")
title("Ball resultant velocity vs time")
extra_graph1_file_name = "%gm_ball_velocity_vs_time.png";
extra_graph1_file_name = sprintf(extra_graph1_file_name, x_specified);
extra_graph1_file_name = fullfile(graph_results_folder, extra_graph1_file_name);
saveas(gcf, extra_graph1_file_name);

% Ball acceleration vs time 
figure
plot(time_vector, ball_acceleration_vector);
ylabel("acceleration (m/s^2)")
xlabel("time (s)")
title("Ball resultant acceleration vs time")
extra_graph2_file_name = "%gm_ball_acceleration_vs_time.png";
extra_graph2_file_name = sprintf(extra_graph2_file_name, x_specified);
extra_graph2_file_name = fullfile(graph_results_folder, extra_graph2_file_name);
saveas(gcf, extra_graph2_file_name);

% Reference vs Actual Position
figure
plot(time_vector, position_ref_vector);
hold on
plot(time_vector, position_vector);
hold off
grid on
legend("position reference", "position actual")
title("Arm position \theta (degrees)")
xlabel("time (s)")
graph2_file_name = "%gm_position_arm_ref_vs_actual.png";
graph2_file_name = sprintf(graph2_file_name, x_specified);
graph2_file_name = fullfile(graph_results_folder, graph2_file_name);
saveas(gcf, graph2_file_name);

% Reference vs Actual Speed
figure
plot(time_vector, speed_ref_vector);
hold on
plot(time_vector, speed_vector);
hold off
grid on
legend("speed reference", "speed actual");
title("Arm speed \omega (RPM)")
xlabel("time (s)")
graph3_file_name = "%gm_speed_arm_ref_vs_actual.png";
graph3_file_name = sprintf(graph3_file_name, x_specified);
graph3_file_name = fullfile(graph_results_folder, graph3_file_name);
saveas(gcf, graph3_file_name);

% Arm acceleration vs time
figure
plot(time_vector, arm_acceleration_vector);
title("Arm acceleration \alpha (m/s^2)")
xlabel("time (s)")
extra_graph3_file_name = "%gm_arm_acceleration_vs_time.png";
extra_graph3_file_name = sprintf(extra_graph3_file_name, x_specified);
extra_graph3_file_name = fullfile(graph_results_folder, extra_graph3_file_name);
saveas(gcf, extra_graph3_file_name);

% Arm input torque vs time
figure
plot(time_vector, input_torque_vector);
title("input motor Torque \tau (N-m)")
xlabel("time (s)")
extra_graph4_file_name = "%gm_input_torque_vs_time.png";
extra_graph4_file_name = sprintf(extra_graph4_file_name, x_specified);
extra_graph4_file_name = fullfile(graph_results_folder, extra_graph4_file_name);
saveas(gcf, extra_graph4_file_name);

% Error Plots
figure;
subplot(2, 2, 1); plot(time_vector, model.error_position.Data); grid on;
title("Position Error")
subplot(2, 2, 2); plot(time_vector, model.error_speed.Data); grid on;
title("Speed Error")
subplot(2, 2, 3); plot(time_vector, model.error_current.Data); grid on;
title("Current Error")
graph4_file_name = "%gm_error_plots.png";
graph4_file_name = sprintf(graph4_file_name, x_specified);
graph4_file_name = fullfile(graph_results_folder, graph4_file_name);
saveas(gcf, graph4_file_name);

% Electrical Plots
figure;
subplot(2, 2, 1); plot(time_vector, power_vector); grid on;
title("Power (W)")
subplot(2, 2, 2); plot(time_vector, current_vector); grid on;
title("Current (A)")
subplot(2, 2, 3); plot(time_vector, voltage_vector); grid on;
title("Voltage (V)")
subplot(2, 2, 4); plot(time_vector, power_consumption_vector); grid on;
title("Power Consumption (mWh)")
graph5_file_name = "%gm_electrical_plots.png";
graph5_file_name = sprintf(graph5_file_name, x_specified);
graph5_file_name = fullfile(graph_results_folder, graph5_file_name);
saveas(gcf, graph5_file_name);

%% Various Calculations

% calculations for power usage
max_power = round(max(power_vector), 2);
consumed_power = round(max(power_consumption_vector), 2);

% calculations related to ball impact
t_data_land = t_data_impact(end);
x_data_land = x_data_impact(end);
y_data_max = max(y_data);

%% display and output information for user

ball_info = ["t_data_land (s): ", t_data_land;
             "x_data_land (m): ", x_data_land;
             "y_data_max (m): ", y_data_max];
         
arm_timer_info = ["t_back_to_rest (s):", back_to_start_timer(end);
                  "angle at this time (deg):", back_to_start_position(end)];
         
power_info = ["Max Power (W):", max_power;
              "Power Consumed (mWh)", consumed_power];

information_array = cat(1, ball_info, arm_timer_info, power_info);
          
disp(information_array);
results_file_name = "%gm_target_results.txt";
results_file_name = sprintf(results_file_name, x_specified);
writematrix(information_array, results_file_name);









