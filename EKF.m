function [X, X_pred, P, R_k] = EKF(X, Y, u, P, Z, Q, dt, L, var_GPS, var_R, alpha, beta)
    % EKF - Extended Kalman Filter for State Estimation
    % 
    % Input:
    %   X - Actual State (4x1)
    %   Y - State of Target vehicle for Relative Measurements (4x1)
    %   u - Actual Control Inputs (2x1)
    %   P - Actual State Covariance (4x4)
    %   Z - Actual Measurements (4x1)
    %   Q - Covariance Matrix of Kinematic Model (4x4)
    %   dt - Time step
    %   L - Vehicle Length
    %   var_GPS - Variance of GPS Measurement System
    %   var_R - Variance of Radar Measurement System
    %   alpha - Scaling Factor of Radar Uncertainty
    %   beta - Trust Factor for Radar Measurement

    % Output:
    %   X - Updated State
    %   P - Updates State Covariance
    %   R_k - Updated Dynamic Measurements Covariance (Uncertainty varying on distance)

    %% --- PREDICTION ---
    X_pred = Car_Like_Model(X, u, dt, L);  % Predicted State using Kinematic Model
   
    % Jacobian A_k (Model Linearization)
    A_k = A(X, u, dt, L);

    % Covariance Prediction
    P_pred = A_k * P * A_k' + Q;
    

    %% R_k Matrix Definition and Radar Uncertainty Modeling
    
    d_k = norm(X_pred(1:2) - Y(1:2)); % Traget Vehicle distance
    
    p_yk = alpha * d_k^beta; % Uncertainty over target position
    
    % Definitive Measuments Covariance Matrix
    R_k = diag([var_GPS, var_GPS, var_R + p_yk, var_R + p_yk]);


    %% --- UPDATE ---

    H_k = [1,0,0,0; 0,1,0,0; -1,0,0,0; 0,-1,0,0]; % Jacobian Matrix of Measurement Model

    % Measurements Prediction
    Z_pred = H_k * X_pred; 
    
    % Innovation
    Y_k = Z - Z_pred;

    % Innovation Covariance
    S_k = H_k * P_pred * H_k' + R_k;

    % Kalman Gain
    K_k = P_pred * H_k' / S_k;

    % State Update
    X = X_pred + K_k * Y_k;

    % Covariance Update
    P = (eye(size(P)) - K_k * H_k) * P_pred;
end


% Function for A matrix calculation
function [A_next] = A(X, u, dt, L)
    A_next = [1, 0, -dt * X(4) * sin(X(3)), dt * cos(X(3));
           0, 1,  dt * X(4) * cos(X(3)), dt * sin(X(3));
           0, 0,  1, dt / (L * cos(u(2))^2);
           0, 0,  0, 1];
end


