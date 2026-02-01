function dxdt = stateFcnRoboticArm(x, u, params)
% stateFcnRoboticArm: Computes the time derivative of the 2-DOF robotic arm state.
%
% Inputs:
%   x      - State vector [theta1; omega1; theta2; omega2]
%            theta1  = Joint 1 angle (rad)
%            omega1  = Joint 1 angular velocity (rad/s)
%            theta2  = Joint 2 angle (rad)
%            omega2  = Joint 2 angular velocity (rad/s)
%   u      - Control input vector [tau1; tau2]
%            tau1    = Joint 1 input torque (N*m)
%            tau2    = Joint 2 input torque (N*m)
%   params - Vector of physical parameters:
%            [m1; m2; a1; a2; kj1; kj2; ac1; ac2]
%            m1   = Mass of link 1 (kg)
%            m2   = Mass of link 2 (kg)
%            a1   = Length of link 1 (m)
%            a2   = Length of link 2 (m)
%            kj1  = Joint 1 viscous friction (N*m*s/rad)
%            kj2  = Joint 2 viscous friction (N*m*s/rad)
%            ac1  = Center of mass position, link 1 (m)
%            ac2  = Center of mass position, link 2 (m)
% Output:
%   dxdt   - Time derivative of state vector [omega1; domega1; omega2; domega2]

theta1 = x(1);      % Joint 1 angle (rad)
omega1 = x(2);      % Joint 1 angular velocity (rad/s)
theta2 = x(3);      % Joint 2 angle (rad)
omega2 = x(4);      % Joint 2 angular velocity (rad/s)
tau1_d = x(5);      % Joint 1 input torque disturbance(N*m)
tau2_d = x(6);      % Joint 2 input torque disturbance(N*m)

tau1 = u(1);        % Torque applied to joint 1 (N*m)
tau2 = u(2);        % Torque applied to joint 2 (N*m)

m1 = params(1);     % Mass of link 1 (kg)
m2 = params(2);     % Mass of link 2 (kg)
a1 = params(3);     % Length of link 1 (m)
a2 = params(4);     % Length of link 2 (m)
kj1 = params(5);    % Viscous friction at joint 1 (N*m*s/rad)
kj2 = params(6);    % Viscous friction at joint 2 (N*m*s/rad)
ac1 = params(8);    % Center of mass position, link 1 (m)
ac2 = params(9);    % Center of mass position, link 2 (m)

I1 = m1 * a1^2 / 12;       % Moment of inertia, link 1 (kg*m^2)
I2 = m2 * a2^2 / 12;       % Moment of inertia, link 2 (kg*m^2)

g = 9.81;                  % Gravity (m/s^2)

C1 = cos(theta1);
C2 = cos(theta2);
C12 = cos(theta1 + theta2);
S2 = sin(theta2);

% Inertia matrix (2x2)
M = [m1*ac1^2 + m2*(a1^2 + ac2^2 + 2*a1*ac2*C2) + I1 + I2, m2*(ac2^2 + a1*ac2*C2) + I2;
     m2*(ac2^2 + a1*ac2*C2) + I2,                         m2*ac2^2 + I2];

% Coriolis/Centrifugal matrix (2x2)
C = [-2*m2*a1*ac2*S2*omega2,           -m2*a1*ac2*S2*omega2;
      m2*a1*ac2*S2*omega1,                      0];

% Gravity vector (2x1)
G = [m1*g*ac1*cos(theta1) + m2*g*(a1*cos(theta1) + ac2*cos(theta1 + theta2));
     m2*g*ac2*cos(theta1 + theta2)];

% Viscous friction (2x1)
f = [kj1*omega1; kj2*omega2];

tau = [tau1 + tau1_d; tau2 + tau2_d];

% Joint angular accelerations (domega)
domega = M \ (tau - C * [omega1; omega2] - G - f);

% Assemble state derivative vector
dxdt = [omega1; domega(1); omega2; domega(2); 0; 0];
end