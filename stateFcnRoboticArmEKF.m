function xk1 = stateFcnRoboticArmEKF(xk, uk)
% stateFcnRoboticArmEKF: Discrete-time update for 2-DOF robotic arm EKF process model.
%
% Inputs:
%   xk    - Current state vector [theta1; omega1; theta2; omega2]
%           theta1:  Joint 1 angle (rad)
%           omega1:  Joint 1 angular velocity (rad/s)
%           theta2:  Joint 2 angle (rad)
%           omega2:  Joint 2 angular velocity (rad/s)
%           tau1_d:  Joint 1 input torque disturbance (N*m)
%           tau2_d:  Joint 2 input torque disturbance (N*m)
%   uk    - Augmented control/parameter vector:
%           uk(1):  tau1   (Joint 1 input torque, N*m)
%           uk(2):  tau2   (Joint 2 input torque, N*m)
%           uk(3:10): params (physical parameters [m1; m2; a1; a2; kj1; kj2; h; ac1; ac2])
%           uk(11): Ts     (Sampling time, s)
%
% Output:
%   xk1   - Predicted next state vector after sampling time Ts

params = uk(3:11);   % Physical parameters vector
Ts = uk(12);         % Sampling time (s)

xk1 = xk + Ts * stateFcnRoboticArm(xk, uk(1:2), params);  % Euler discretization
end