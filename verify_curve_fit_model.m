
x_distances_specified = linspace(0.3, 1.5, 13);
x_distances_actual = zeros(1, length(x_distances_specified));
for index = 1:length(x_distances_specified)
    pause(5);
    x_specified = x_distances_specified(index);
    final_simulation
    x_distances_actual(index) = x_data_land;
end
	
csv_array = [x_distances_specified', x_distances_actual'];
writematrix(csv_array, "verify_model.csv");