
clear all
close all
clc

x_distances_specified = linspace(0.2, 1.5, 131);
% x_distances_specified = [0.2, 0.5, 1.5];
x_distances_actual = zeros(1, length(x_distances_specified));
t_retract_results = zeros(1, length(x_distances_specified));
t_retract_angle_results = zeros(1, length(x_distances_specified));

for index = 1:length(x_distances_specified)
    pause(5);
    x_specified = x_distances_specified(index);
    final_simulation
    x_distances_actual(index) = x_data_land;
    t_retract_results(index) = t_retract;
    t_retract_angle_results(index) = t_retract_angle;
end
	
csv_array = [x_distances_specified', x_distances_actual', t_retract_results', t_retract_angle_results'];
writematrix(csv_array, "verify_model.csv");
