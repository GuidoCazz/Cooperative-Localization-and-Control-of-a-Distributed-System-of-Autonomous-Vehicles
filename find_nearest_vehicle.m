% Function calculating the nearest Vehicle given one
function nearest_idx = find_nearest_vehicle(pos, X)
    distances = vecnorm(X(1:2,:) - pos, 2, 1);
    distances(distances == 0) = inf;
    [~, nearest_idx] = min(distances); 
end