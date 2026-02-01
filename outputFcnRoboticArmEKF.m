function y = outputFcnRoboticArmEKF(xk, params)
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
%                x(5): tau1_d    (joint 1 input torque disturbance in N*m)
%                x(6): tau2_d    (joint 2 input torque disturbance in N*m)
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

y = outputFcnRoboticArm(xk, [], params);

end

