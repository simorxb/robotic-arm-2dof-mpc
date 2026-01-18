%% Robotic arm parameters
% Base size
lbx = 0.1;
lby = 0.1;
lbz = 0.4;

% Joint 1
lj1 = 0.08;
rj1 = 0.02;
kj1 = 0.002;

% Joint 2
lj2 = 0.06;
rj2 = 0.02;
kj2 = 0.002;

% End effector
le1 = 0.05;
re1 = 0.012;
le2 = 0.04;
re2 = 0.002;

% Arm lengths
a1 = 0.2;
r1 = 0.01;
a2 = 0.15;
r2 = 0.01;

% Steel density
rho_steel = 7850;

% Masses
ml1 = rho_steel * pi * r1^2 * a1;
ml2 = rho_steel * pi * r2^2 * a2;
mj2 = rho_steel * pi * rj2^2 * lj2;
me1 = rho_steel * pi * re1^2 * le1;
me2 = rho_steel * pi * re2^2 * le2;

m1 = ml1 + mj2;
m2 = ml2 + me1 + me2;

% Centers of mass
ac1 = (ml1*a1/2 + mj2*a1) / (ml1 + mj2);
ac2 = (ml2*a2/2 + me1*a2 + me2*a2) / (ml2 + me1 + me2);

% Initial position
theta1_0 = -pi/2;
theta2_0 = 0;