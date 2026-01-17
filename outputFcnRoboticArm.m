function y = outputFcnRoboticArm(x, ~, params)
%OUTPUTFCNHOVERCRAFT Output function for the hovercraft system.
%   y = OUTPUTFCNHOVERCRAFT(x, ~, ~) returns the output variable y for the
%   given state vector x.
%
%   Input:
%       x - State vector where:
%           x(1): theta    (heading angle in radians)
%           x(2): dtheta   (angular velocity in radians/sec)
%           x(3): x_pos    (x position in meters)
%           x(4): dx       (x velocity in m/s)
%           x(5): y_pos    (y position in meters)
%           x(6): dy       (y velocity in m/s)
%
%   Output:
%       y - Output vector [cos(theta); sin(theta); x_pos; y_pos] for hovercraft state (all in global frame)

theta1 = x(1);      % Joint 1 angle (rad)
theta2 = x(3);      % Joint 2 angle (rad)

a1 = params(3);     % Length of link 1 (m)
a2 = params(4);     % Length of link 2 (m)
h = params(7);     % Height of arm base

xe = a1*cos(theta1) + a2*cos(theta2);
ye = h + a1*sin(theta1) + a2*sin(theta2);

y = [xe; ye];

end

