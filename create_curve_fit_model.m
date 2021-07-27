close all
clc

curve_fit_data = readmatrix("curve_fit_model_data_2_5.csv");
y_swing_angles = curve_fit_data(:, 1);
x_x_distances = curve_fit_data(:, 2);
[x_x_distances, sortIdx] = sort(x_x_distances, "ascend");
y_swing_angles = y_swing_angles(sortIdx);

polynomial_coeffs = polyfit(x_x_distances, y_swing_angles, 6);
y_curve_fit_model = polyval(polynomial_coeffs, x_x_distances);
writematrix(polynomial_coeffs, "curve_fit_model_2_5.csv")

plot(x_x_distances, y_curve_fit_model)

angle_predict = polyval(polynomial_coeffs, 0.75);