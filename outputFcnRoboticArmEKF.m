function y = outputFcnRoboticArmEKF(xk, params)
%OUTPUTFCNROBOTICARMEKF Output function for the 2-DOF robotic arm system (EKF version).
%   y = OUTPUTFCNROBOTICARMEKF(xk, params) returns the output variable y for the
%   given state vector xk and parameter vector params.
%
%   Input:
%       xk     - State vector where:
%                xk(1): theta1    (joint 1 angle in radians)
%                xk(2): omega1    (joint 1 angular velocity in radians/sec)
%                xk(3): theta2    (joint 2 angle in radians)
%                xk(4): omega2    (joint 2 angular velocity in radians/sec)
%                xk(5): tau1_d    (joint 1 input torque disturbance in N*m)
%                xk(6): tau2_d    (joint 2 input torque disturbance in N*m)
%
%       params - Parameter vector where:
%                params(3): a1   (length of link 1 in meters)
%                params(4): a2   (length of link 2 in meters)
%                params(7): h    (height of arm base in meters)
%
%   Output:
%       y - Output vector [ye; ze], where:
%           ye = y position of end-effector (in base frame)
%           ze = z position of end-effector (in base frame)

y = outputFcnRoboticArm(xk, [], params);

end

