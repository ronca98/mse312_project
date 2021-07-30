
clear all
close all
clc

x_distances_specified = linspace(0.2, 1.5, 111);
% x_distances_specified = [0.26, 0.63, 1.5, 1.89, 1.41];
x_distances_actual = zeros(1, length(x_distances_specified));
timer_results = zeros(1, length(x_distances_specified));
timer_results_position = zeros(1, length(x_distances_specified));

for index = 1:length(x_distances_specified)
    pause(5);
    x_specified = x_distances_specified(index);
    final_simulation
    x_distances_actual(index) = x_data_land;
    timer_results(index) = back_to_start_timer(end);
    timer_results_position(index) = back_to_start_position(end);
end
	
csv_array = [x_distances_specified', x_distances_actual', timer_results', timer_results_position'];
writematrix(csv_array, "verify_model_test_pts_long.csv");
