function [X_tri, Y_tri] = RobotFigure(X, scale)
% RobotFigure - Animation of robot
    % 
    % Input:
    %   X - Actual State (4x1)
    %   scale - Scaling factor for robot dimensions

    % Output:
    %   X_tri - Figure Output X coordinates
    %   Y_tri - Figure Output Y coordinates
    
    p1 = [X(1) + scale * cos(X(3)), X(2) + scale * sin(X(3))];
    p2 = [X(1) + scale * cos(X(3) + 5 * pi / 6), X(2) + scale * sin(X(3) + 5 * pi / 6)];
    p3 = [X(1) + scale * cos(X(3) - 5 * pi / 6), X(2) + scale * sin(X(3) - 5 * pi / 6)];
    
    X_tri = [p1(1), p2(1), p3(1)];
    Y_tri = [p1(2), p2(2), p3(2)];
end