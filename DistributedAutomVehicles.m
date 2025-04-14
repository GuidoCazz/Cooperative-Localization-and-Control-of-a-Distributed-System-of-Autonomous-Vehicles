% Main Distributed System Script

clear; clc; close all;
%% ---- PARAMETERS ----

dt = 0.1;   % Time step
N = 300;    % Number of iterations
L = 2.5;    % Vehicle length
N_vehicles = 4; 
d_safe = 30;    % Distance of Warning


% Parameters for PID of velocity

v_ref = 12;    % Reference velocity

Kp1 = 1.5;  % Proportional Gain over Security Distance
Kp2 = 0.5;  % Proportional Gain Velocity
Kd = 0.01;  % Derivative Gain
Ki = 0.1;   % Integral Gain
K_fuel = 0.5;   % Fuel Consumption Gain

integral_v = zeros(1, N_vehicles);
prev_error_v = zeros(1, N_vehicles);

% Simplified MOE model coefficients for fuel consumption
K00 = 0.679439;
K01 = 0.135273;
K10 = 0.029665;

fuel_consumption = zeros(1, N_vehicles);
fuel_total = 0;
time_steps = 0;

% Stanley Controller of steering angle
K_stanley = 0.1; 

% Consensus weight
q = 0.01;

% Environment Limits
x_min = -100;
x_max = 100;
y_min = -100;
y_max = 100;

waypoints = [
    x_min + d_safe, y_min + d_safe;
    x_max - d_safe, y_min + d_safe;
    x_max - d_safe, y_max - d_safe;
    x_min + d_safe, y_max - d_safe;
    x_min + d_safe, y_min + d_safe 
];

% Trajectory Generation
num_points = 100; 
traj_x = interp1(1:5, waypoints(:,1), linspace(1,5,num_points));
traj_y = interp1(1:5, waypoints(:,2), linspace(1,5,num_points));

dx = diff(traj_x);  % Local Line Coefficients
dy = diff(traj_y);
traj_theta  = atan2(dy, dx);
traj_theta = [traj_theta traj_theta(end)]; 



% Initial States [x, y, theta, v]
X = [  0,  75, -2,  40;
      0,  60,  1/2, 2;
       20, 65,  2,  15;
      -5,   70, -1,  5]';

% Initial Control Inputs (acceleration, steering angle)
u = [  1,  2, -2,  3;
      0,  0,  0, 0];

% Coefficients for Radar Variance modeling
alpha = 0.05;
beta = 1;

% Initial System Covariance
P = repmat(diag([2, 2, 0.5, 1]), 1, 1, 4);

% Initial Covariances
Q = diag([0.01, 0.01, 0.001, 0.001]);
var_GPS = 2.25;
var_R = 0.56;
Pd = 1;
R_k = diag([var_GPS, var_GPS, var_R + Pd, var_R + Pd]);


% Communication Parameters
comm_range = 300;  % Maximum communication range in meters
packet_loss_prob = 0.01;        % Probability of message being lost
comm_delay = 2;                % Delay in timesteps
comm_buffer = repmat({zeros(1, N_vehicles)}, 1, comm_delay);  % Circular buffer for delayed velocity info

% Check communication capability
can_communicate = @(pos1, pos2, range) norm(pos1 - pos2) <= range;

%% Trajectories and Plot Initialization

% Environment Plot
figure; hold on; grid on; axis equal;
title('Simulation Environment');
xlabel('X Position (m)');
ylabel('Y Position (m)');

% Borders
plot([x_min x_max x_max x_min x_min], ...
     [y_min y_min y_max y_max y_min], 'k-', 'LineWidth', 2);

% Waypoints
plot(waypoints(:,1), waypoints(:,2), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);

% Traiettoria interpolata
plot(traj_x, traj_y, 'b--', 'LineWidth', 1.5);

legend('Environment Boundary', 'Waypoints', 'Interpolated Path');

trajectory = zeros(4, N, N_vehicles);
trajectory(:,1,:) = X;
X_true_store = zeros(4, N, N_vehicles);  % Real trajectories
X_est_store = zeros(4, N, N_vehicles);   % Estimated trajectories
X_true_store(:,1,:) = X;
X_est_store(:,1,:) = X;
velocity = zeros(N, N_vehicles);
distances = zeros(N, N_vehicles);


% Total Fuel Consumption
figure;
h_fuel = plot(0, 0, 'r', 'LineWidth', 2);
xlabel('Time Steps');
ylabel('Total Fuel Consumption [L]');
title('Fuel Consumption Over Time [L]');
grid on;
hold on;

% Instantaneous Fuel Consumption
figure;
h_fuel_inst = plot(0, 0, 'b', 'LineWidth', 2);
xlabel('Time Steps');
ylabel('Instantaneous Fuel Consumption [mL/s]');
title('Instantaneous Fuel Consumption Over Time [mL/s]');
grid on;
hold on;

% System Inizialization
figure;
hold on; grid on;
xlabel('X Position (m)'); ylabel('Y Position (m)');
title('Extended Kalman Filter (EKF) - Dynamic Plot');
axis equal;
xlim([x_min x_max]); ylim([y_min y_max]);

plot([-100 -100 100 100 -100], [-100 100 100 -100 -100], 'k-', 'LineWidth', 2);
colors = {'b', 'g', 'r', 'c'};
kf_plot = gobjects(1,N_vehicles);
h_robot = gobjects(1,N_vehicles); % Handle creation for every robot

% Initializing robot model
for i = 1:N_vehicles
    kf_plot(i) = plot(trajectory(1,1,i), trajectory(2,1,i), colors{i}, 'LineWidth', 2);
    [X_tri, Y_tri] = RobotFigure(X(:,i),3); 
    h_robot(i) = patch(X_tri, Y_tri, colors{i});
end

%% ---- EXTENDED KALMAN FILTER LOOP ----
measurement_update_interval = 1; % Meaurement update rate

for k = 2:N
    % GPS and Radar measurements initialization
    if mod(k, measurement_update_interval) == 0 || k == 2
        for i = 1:N_vehicles
            Z_noise = normrnd(0, sqrt(diag(R_k)));  % Noise 
            Z(:,i) = Measurement_Model(X(:,i)) + Z_noise; % Measurement Model
        end
    end

    fuel_step = 0;  % Instantaneous fuel consumption initialization

    % Estimation for all vehicles 
    for i = 1:N_vehicles
        
        %% ---- COMMUNICATION MODEL ----

        % Get delayed states from buffer
        delayed_velocities = comm_buffer{1};

        % Determine reachable and successful neighbors
        neighbors = [];
        for j = 1:N_vehicles
            if j ~= i && can_communicate(X(1:2,i), X(1:2,j), comm_range)
                if rand > packet_loss_prob
                    neighbors(end+1) = j;              
                end
            end
        end
        
        % Apply velocity consensus only with reachable neighbors
        if ~isempty(neighbors)
            X(4,i) = Vel_Consensus(delayed_velocities, X(4,i), neighbors, q);
        end
    
        % PID Controller over Velocity (Acceleration)
        nearest_vehicle_idx = find_nearest_vehicle(X(1:2,i), X);
        [u(1,i), integral_v(i), prev_error_v(i)] = PID_Controller(X(:,i), X(:,nearest_vehicle_idx), v_ref, integral_v(i), prev_error_v(i), dt, Kp1, Kp2, Kd, Ki, K_fuel, d_safe);
       
        % Stanley controller over Path (Steering Angle)
        u(2,i) = Stanley_controller(X(:,i), traj_x, traj_y, traj_theta, K_stanley);

        if i == N_vehicles
            n = 1;
        else
            n = i+1;
        end
        
        % Extended Kalman filter
        [X(:,i), X_true_store(:,k,i), P(:,:,i), R_k] = EKF(X(:,i), X(:,n), u(:,i), P(:,:,i), Z(:,i), Q, dt, L, var_GPS, var_R, alpha, beta);
        X_est_store(:,k,i) = X(:,i); 
        
        % Velocity and Distances Storage
        velocity(k,i) = X(4,i);
        distances(k,i) = norm(X(1:2,i) - X(1:2,nearest_vehicle_idx));
        

        % Error between estimated and actual trajectory (for RMSE)
        error_x(k,i) = X_est_store(1,k,i) - X_true_store(1,k,i);
        error_y(k,i) = X_est_store(2,k,i) - X_true_store(2,k,i);

        % Fuel Consumption Estimation
        fuel_step = fuel_step + (K00 + K01 * abs(u(1,i)) + K10 * abs(X(4,i))) * 1e-3 * dt;

        % Updating Vehicles Position
        set(kf_plot(i), 'XData', [get(kf_plot(i), 'XData'), X(1,i)], ...
                'YData', [get(kf_plot(i), 'YData'), X(2,i)]);
        
        [X_tri, Y_tri] = RobotFigure(X(:,i),3);
        set(h_robot(i), 'XData', X_tri, 'YData', Y_tri);
        drawnow;
    end
    
    % Update communication buffer (FIFO for delay)
    comm_buffer = [comm_buffer(2:end), {X(4,:)}];

    % Updating Fuel Consumption Plots
    fuel_total = fuel_total + fuel_step;
    set(h_fuel, 'XData', [get(h_fuel, 'XData'), k*dt], 'YData', [get(h_fuel, 'YData'), fuel_total]);
    set(h_fuel_inst, 'XData', [get(h_fuel_inst, 'XData'), k*dt], 'YData', [get(h_fuel_inst, 'YData'), fuel_step * 1e3]);
    
    
    pause(0.1);
end

figure;
hold on;
% Calculation of overall RMSE
for i = 1:N_vehicles
    RMSEx(i) = sqrt(mean(error_x(:,i).^2));
    RMSEy(i) = sqrt(mean(error_y(:,i).^2));

    plot(1:N, velocity(:,i));   % Overall velocity plot     
end

figure;
hold on;
for i=1:N_vehicles
    plot(1:N, distances(:,i), 'b', 'LineWidth', 2);    % Overall vehicle's distance plot
end