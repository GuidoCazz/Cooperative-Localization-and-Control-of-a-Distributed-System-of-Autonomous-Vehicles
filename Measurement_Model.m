function Z = Measurement_Model(X)

    % Direct GPS Measurements of X and Y Position
    Z_gps = [X(1); X(2)];
    
    % Relative measurent of distance subtracted by Target Vehicle position
    Z_radar = [-X(1); -X(2)];
    
    % Vector of combined measurements
    Z = [Z_gps; Z_radar];
end