function y = outputFcnRoboticArm(x, ~, params)
%OUTPUTFCNROBOTICARM Output function for the 2-DOF robotic arm system.
%   y = OUTPUTFCNROBOTICARM(x, ~, params) returns the output variable y for the
%   given state vector x and parameter vector params.
%
%   Input:
%       x      - State vector where:
%                x(1): theta1    (joint 1 angle in radians)
%                x(2): omega1    (joint 1 angular velocity in radians/sec)
%                x(3): theta2    (joint 2 angle in radians)
%                x(4): omega2    (joint 2 angular velocity in radians/sec)
%
%       params - Parameter vector where:
%                params(3): a1   (length of link 1 in meters)
%                params(4): a2   (length of link 2 in meters)
%                params(7): h    (height of arm base in meters)
%
%   Output:
%       y - Output vector [ye; ze] where:
%           ye = x position of end-effector (in base frame)
%           ze = z position of end-effector (in base frame)

theta1 = x(1);      % Joint 1 angle (rad)
theta2 = x(3);      % Joint 2 angle (rad)

a1 = params(3);     % Length of link 1 (m)
a2 = params(4);     % Length of link 2 (m)
h = params(7);     % Height of arm base

ye = a1*cos(theta1) + a2*cos(theta1 + theta2);
ze = h + a1*sin(theta1) + a2*sin(theta1 + theta2);

y = [ye; ze];

end

