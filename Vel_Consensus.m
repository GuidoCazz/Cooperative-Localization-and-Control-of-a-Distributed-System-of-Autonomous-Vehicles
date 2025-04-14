function [v_cons] = Vel_Consensus(Xv, vehicle_vel, neighbors, q)
% Vel_Consensus - Application of consensus algorithm over velocity  
    % 
    % Input:
    %   Xv - Vector containing all vehicles velocities (1x4)
    %   vehicle_vel - Current vehicle velocity
    %   neighbors - Indices of neighboring vehicles (1x3)
    %   q - Consensus gain factor

    % Output:
    %   v_cons - Updated consensus velocity

        v_neighbors = mean(Xv(neighbors));
        v_cons = vehicle_vel + q * (v_neighbors - vehicle_vel);
end