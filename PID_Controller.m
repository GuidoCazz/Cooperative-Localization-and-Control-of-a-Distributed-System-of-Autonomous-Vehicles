function [a, integral_new, error_new] = PID_Controller(X, X_next, v_ref, integral_old, prev_error, dt, Kp1, Kp2, Kd, Ki, K_fuel, d_safe)
% PID_Controller - Feedback Control of vehicles acceleration 
    % 
    % Input:
    %   X - Actual State (4x1)
    %   X_next - Nearest Vehicle Actual State (4x1)
    %   v_ref - Reference Speed
    %   integral_old - Previous integrative error
    %   prev_error - Prevous velocity error
    %   dt - Time step
    %   Kp1 - Proportional Gain over Security Distance
    %   Kp2 - Proportional Velocity Gain 
    %   Kd - Derivative Gain
    %   Ki - Integral Gain
    %   K_fuel - Fuel Consumption Gain
    %   d_safe - Distance of Alert

    % Output:
    %   a - New Input Acceleration
    %   integral_new - New integrative error
    %   error_new - New velocity error
    
    
    rel_pos = X_next(1:2) - X(1:2); % Relative position vector
    direction = [cos(X(3)); sin(X(3))]; % Direction of current vehicle

    distance = norm(rel_pos); % Vehicle Distance
    
    % Penalization on distance between the vehicles (repulsive acceleration)
    if distance < d_safe

        if dot(rel_pos, direction) < 0            
            error_d = (d_safe - distance)/d_safe; % Vehicle is ahead
        else
            error_d = - (d_safe - distance)/d_safe; % Vehicle is behind
        end

    else
        error_d = 0;
    end

    % Error on difference of speed
    error = v_ref - X(4);

    % Integral error update on velocity
    integral_new = integral_old + error * dt;

    % Derivate of speed error
    derivative = (error - prev_error) / dt;

    % Control command computation
    a = Kp1 * error_d + Kp2 * error + Ki * integral_new + Kd * derivative - K_fuel * abs(error); 

    error_new = error;
end
