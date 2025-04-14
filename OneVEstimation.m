% One Vehicle estimation (showing effects of measurements and EKF estimation)

clear; clc; close all;

%% ---- PARAMETERS ----

dt = 0.1;  % Time step
N = 50;   % Number of iterations
L = 2.5;   % Vehicle length

% Initial States [x, y, theta, v]
X1 = [0; 0; 0; 2];  % Main Robot
X2 = [10; 5; pi/6; 2];  % Radar Reference

% Initial Control Inputs (acceleration, steering angle)
u1 = [0.1; 0.05];
u2 = [0.05;  0.1];

% Coefficients for Radar Variance modeling
alpha = 0.05;
beta = 1.5;

% Initial System Covariance
P = diag([1, 1, 0.1, 0.5]);

% Initial Covariances
Q = diag([0.05, 0.05, 0.01, 0.02]);   
var_GPS = 0.5;
var_R = 0.2;
Pd = 0.1;
R_k = diag([var_GPS, var_GPS, var_R + Pd, var_R + Pd]);


%% Trajectories and Plot Initialization

actual_trajectory1 = zeros(2, N);
actual_trajectory2 = zeros(2, N);
estimated_trajectory1 = zeros(2, N);
estimated_trajectory2 = zeros(2, N);
gps_measures = zeros(2, N);
radar_measures = zeros(2, N);

% Initial States for simulation
X1_real = X1;
X2_real = X2;
X1_est = X1;  
X2_est = X2;

figure;
hold on; grid on; axis equal;
xlabel('X Position (m)'); ylabel('Y Position (m)');
title('EKF Vehicle Estimation - Two Robots');
xlim([-2 50]); ylim([-2 50]);
% Trajectory Plot
actual_plot1 = plot(NaN, NaN, 'k--', 'LineWidth', 2, 'DisplayName', 'Actual Trajectory 1');
actual_plot2 = plot(NaN, NaN, 'm--', 'LineWidth', 2, 'DisplayName', 'Actual Trajectory 2');
estimated_plot1 = plot(NaN, NaN, 'b-', 'LineWidth', 2, 'DisplayName', 'EKF Estimation 1');
gps_plot = plot(NaN, NaN, 'rx', 'MarkerSize', 8, 'DisplayName', 'GPS Measures');
radar_plot = plot(NaN, NaN, 'go', 'MarkerSize', 8, 'DisplayName', 'Radar Measures');

% Robot Shape
[X_tri1, Y_tri1] = RobotFigure(X1_real,2);
robot_patch1 = patch(X_tri1, Y_tri1, 'r');
[X_tri2, Y_tri2] = RobotFigure(X2_real,2);
robot_patch2 = patch(X_tri2, Y_tri2, 'g');

%% ---- EXTENDED KALMAN FILTER LOOP ----

for k = 2:N

    % Real Vehicle evolution
    X1_real = Car_Like_Model(X1_real, u1, dt, L);
    X2_real = Car_Like_Model(X2_real, u2, dt, L);
    
    actual_trajectory1(:, k) = X1_real(1:2);
    actual_trajectory2(:, k) = X2_real(1:2);
    
    % Generazione misure GPS e Radar con rumore
    Z = Measurement_Model(X1_real) + normrnd(0, sqrt(diag(R_k)));

    % EKF Update for Main Robot
    [X1_est, P, R_k] = EKF(X1_est, X2_est, [u1; u2], P, Z, Q, dt, L, var_GPS, var_R, alpha, beta);

    % Trajectory and measures update
    estimated_trajectory1(:, k) = X1_est(1:2);
    gps_measures(:, k) = Z(1:2);
    radar_measures(:, k) = -Z(3:4); 

    %% ---- PLOT UPDATE ----
    set(actual_plot1, 'XData', actual_trajectory1(1, 1:k), 'YData', actual_trajectory1(2, 1:k));
    set(actual_plot2, 'XData', actual_trajectory2(1, 1:k), 'YData', actual_trajectory2(2, 1:k));
    set(estimated_plot1, 'XData', estimated_trajectory1(1, 1:k), 'YData', estimated_trajectory1(2, 1:k));
    set(gps_plot, 'XData', gps_measures(1, 1:k), 'YData', gps_measures(2, 1:k));
    set(radar_plot, 'XData', radar_measures(1, 1:k), 'YData', radar_measures(2, 1:k));

    % Aggiornamento della figura dei robot
    [X_tri1, Y_tri1] = RobotFigure(X1_real,2);
    set(robot_patch1, 'XData', X_tri1, 'YData', Y_tri1);
    [X_tri2, Y_tri2] = RobotFigure(X2_real,2);
    set(robot_patch2, 'XData', X_tri2, 'YData', Y_tri2);

    error_x(k) = X1_real(1) - X1_est(1);
    error_y(k) = X1_real(2) - X1_est(2);

    drawnow;
    pause(0.1);
end
rmse_x = sqrt(mean(error_x.^2));
rmse_y = sqrt(mean(error_y.^2));

legend('show');
