

clear all
clc
close all

swing_angles = linspace(-79, -81, 3);
x_distances = zeros(1, length(swing_angles));
y_heights = zeros(1, length(swing_angles));
for index = 1:length(swing_angles)
    pause(5);
    arm_swing_angle = swing_angles(index);
    final_simulation
    x_distances(index) = x_data_land;
    y_heights(index) = y_data_max;
end
	
csv_array = [swing_angles', x_distances' , y_heights'];
writematrix(csv_array, "farthest_distance_data.csv");
