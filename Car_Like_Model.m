function X_next = Car_Like_Model(X, u, dt, L)
% Car_Like_Model - State Estimation based on Car-Like Kinematic Model
    % 
    % Input:
    %   X - Actual State (4x1)
    %   u - Actual Control Inputs (2x1)
    %   dt - Time step
    %   L - Vehicle Length
    
    % Output:
    %   X_next - State Prediction (4x1)

    % Update Model
    x_next = X(1) + dt * X(4) * cos(X(3));
    y_next = X(2) + dt * X(4) * sin(X(3));
    theta_next = X(3) + dt * (X(4)/L) * tan(u(2));
    v_next = X(4) + dt * u(1);
    
    % Update State
    X_next = [x_next; y_next; theta_next; v_next];
end
