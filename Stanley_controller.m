function [delta] = Stanley_controller(X, traj_x, traj_y, traj_theta, K_stanley)
    % Stanley_controller - Feedback Control of vehicles steering angle  
    % 
    % Input:
    %   X - Actual State (4x1)
    %   traj_x - X coordinates of the reference trajectory
    %   traj_y - Y coordinates of the reference trajectory
    %   traj_theta - Orientation angles of the reference trajectory
    %   K_stanley - Gain for the Stanley controller
    
    % Output:
    %   delta - New Input Steering Angle


    % Find the closest point on the reference trajectory
    x = X(1);
    y = X(2);
    theta = X(3);
    
    distances = sqrt((traj_x - x).^2 + (traj_y - y).^2);
    [~, idx] = min(distances);
    
    % Target point on the trajectory
    x_ref = traj_x(idx);
    y_ref = traj_y(idx);
    theta_ref = traj_theta(idx);
    
    % Lateral error computation (cross-track error)
    cross_track_error = (x - x_ref) * sin(theta_ref) - (y - y_ref) * cos(theta_ref);
    
    % Heading error computation (difference between desired and actual orientation)
    theta_error = atan2(sin(theta_ref - theta), cos(theta_ref - theta));

    % Steering angle computation using Stanley control law
    delta = theta_error + atan(K_stanley * cross_track_error / (X(4) + 1e-3)); 

    % Steering angle saturation (limit to ±50 degrees)    
    delta = max(min(delta, deg2rad(50)), -deg2rad(50));
end
