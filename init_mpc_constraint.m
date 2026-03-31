%% Nonlinear Model Predictive Controller (NMPC) Configuration for Robotic Arm

model = 'control_constraint';

% Number of states (theta1, omega1, theta2, omega2, tau1_d, tau2_d)
nx = 6;
% Number of outputs (end-effector x, y)
ny = 2;
% Number of manipulated variables (joint torques)
nmv = 2;
% No unmeasured or measured disturbances for this robotic arm
nu = nmv;

% Create nonlinear MPC controller object
nlobj = nlmpc(nx, ny, 'MV', 1:nmv);

% Number of model parameters (for custom state function)
nlobj.Model.NumberOfParameters = 1;
% Physical parameters vector: [m1; m2; a1; a2; kj1; kj2; h; ac1; ac2]
% Values assumed initialized elsewhere
params = [m1; m2; a1; a2; kj1; kj2; lbz-rj1; ac1; ac2];

% Open the model and create the parameter bus
open([model '.slx']);
clear paramsBusObject;
createParameterBus(nlobj,[model '/Nonlinear MPC Controller'],'paramsBusObject',{params});

% Controller sample time (s)
nlobj.Ts = 0.01;

% Prediction and control horizons
nlobj.PredictionHorizon = 20;
nlobj.ControlHorizon = 5;

% Assign state and output functions
nlobj.Model.StateFcn = "stateFcnRoboticArm";
nlobj.Model.OutputFcn = "outputFcnRoboticArm";

% Rectangular forbidden zone in the (y, z) plane:
rect_xcenter = 0.15;
rect_ycenter = 0.2;
rect_zcenter = 0.25;
rect_lx = 0.2;
rect_ly = 0.2;
rect_lz = 0.5;
margin = 0.01;

rectBounds = [rect_ycenter - rect_ly/2 - margin, rect_ycenter + rect_ly/2 + margin, rect_zcenter - rect_lz/2 - margin, rect_zcenter + rect_lz/2 + margin]; % [y_min, y_max, z_min, z_max]
nlobj.Optimization.CustomIneqConFcn = ...
    @(X, U, e, data, p) rectangularObstacleConstraint(X, U, e, data, p, rectBounds);

% Input (torque) constraints
nlobj.ManipulatedVariables(1).Max = 10;  % Max torque joint 1 (N*m)
nlobj.ManipulatedVariables(1).Min = -10; % Min torque joint 1 (N*m)
nlobj.ManipulatedVariables(2).Max = 10;  % Max torque joint 2 (N*m)
nlobj.ManipulatedVariables(2).Min = -10; % Min torque joint 2 (N*m)

nlobj.ManipulatedVariables(1).ScaleFactor = 2;
nlobj.ManipulatedVariables(2).ScaleFactor = 2;

nlobj.OutputVariables(1).ScaleFactor = 1; % End-effector position x (m)
nlobj.OutputVariables(2).ScaleFactor = 1; % End-effector position y (m)

% Weights for cost function (tracking end-effector, penalize torque rates)
nlobj.Weights.OutputVariables = [1 1];
nlobj.Weights.ManipulatedVariables = [0.0 0.0];
nlobj.Weights.ManipulatedVariablesRate = [0.05 0.05];

% Initial conditions for validation
x0 = [theta1_0; 0; theta2_0; 0; 0; 0];        % [theta1; omega1; theta2; omega2; tau1_d; tau2_d]
u0 = [0 0];            % [tau1; tau2]
validateFcns(nlobj, x0, u0, [], {params});

%% Debug

% Initial states and references for the robotic arm
x = [theta1_0; 0; theta2_0; 0; 0; 0];           % [theta1; omega1; theta2; omega2; tau1_d; tau2_d]
u = [0 0];               % [tau1; tau2]
 
% Reference: end-effector position (y, z)
ye_ref = 0.15;
ze_ref = 0.6;
ref = [ye_ref ze_ref];

nloptions = nlmpcmoveopt;
nloptions.Parameters = {params};

[mv,nloptions] = nlmpcmove(nlobj, x, u, ref, [], nloptions);

% Plot one MPC prediction horizon in the (y,z) plane.
Xpred = [x'; nloptions.X0];
theta1_pred = Xpred(:,1);
theta2_pred = Xpred(:,3);
y_pred = a1*cos(theta1_pred) + a2*cos(theta1_pred + theta2_pred);
z_pred = (lbz - rj1) + a1*sin(theta1_pred) + a2*sin(theta1_pred + theta2_pred);

figure;
plot(y_pred, z_pred, 'bo-', 'LineWidth', 1.5, 'MarkerSize', 4);
hold on;
plot(ye_ref, ze_ref, 'rx', 'LineWidth', 2, 'MarkerSize', 10);
rectangle('Position', [rect_ycenter - rect_ly/2 - margin, rect_zcenter - rect_lz/2 - margin, rect_ly, rect_lz], ...
    'EdgeColor', [0.85 0.2 0.2], 'LineWidth', 1.5, 'LineStyle', '--');
grid on;
axis equal;
xlabel('y (m)');
ylabel('z (m)');
title('Single NMPC Prediction Horizon');
legend('Predicted horizon', 'Reference', 'Location', 'best');
hold off;


%% Simulate MPC for Robotic Arm

% Initial states and references for the robotic arm
x = [theta1_0; 0; theta2_0; 0; 0; 0];           % [theta1; omega1; theta2; omega2; tau1_d; tau2_d]
u = [0 0];               % [tau1; tau2]
 
% Reference: end-effector position (y, z)
ye_ref = 0.15;
ze_ref = 0.6;
ref = [ye_ref ze_ref];

Tend = 1;
steps = round(Tend/nlobj.Ts);

nloptions = nlmpcmoveopt;
nloptions.Parameters = {params};

x_log = zeros(steps, nx);
u_log = zeros(steps, nu);
time_log = zeros(steps, 1);

for i = 1:steps
    x_log(i,:) = x;
    time_log(i) = (i-1)*nlobj.Ts;
    [mv,nloptions] = nlmpcmove(nlobj, x, u, ref, [], nloptions);
    u_log(i,:) = mv';
    
    x = nloptions.X0(1,:);
    u = mv';

    % Print percentage of completion
    percent_done = (i / steps) * 100;
    fprintf('Simulation progress: %.1f%%\n', percent_done);
end

%% Plot results
theta1 = x_log(:,1);
theta2 = x_log(:,3);
y_eff = a1*cos(theta1) + a2*cos(theta1 + theta2);
z_eff = lbz - rj1 + a1*sin(theta1) + a2*sin(theta1 + theta2);

tau1 = u_log(:,1);
tau2 = u_log(:,2);

figure;

subplot(3,2,1)
hold on;
stairs(time_log, y_eff, 'b-', 'LineWidth', 2);
stairs(time_log, ref(1)*ones(size(y_eff)), 'r--', 'LineWidth', 2);
hold off;
grid on;
legend('y', 'y_{ref}');
xlabel('Time (s)');
ylabel('Position (m)');
title('End-Effector y');

subplot(3,2,2)
hold on;
stairs(time_log, z_eff, 'b-', 'LineWidth', 2);
stairs(time_log, ref(2)*ones(size(z_eff)), 'r--', 'LineWidth', 2);
hold off;
grid on;
legend('z', 'z_{ref}');
xlabel('Time (s)');
ylabel('Position (m)');
title('End-Effector z');

subplot(3,2,3)
hold on;
stairs(time_log, theta1*180/pi, 'b-', 'LineWidth', 2);
stairs(time_log, theta2*180/pi, 'g-', 'LineWidth', 2);
hold off;
grid on;
legend('\theta_1', '\theta_2');
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Joint Angles');

subplot(3,2,4)
plot(y_eff, z_eff, 'k-', 'LineWidth', 2);
hold on;
plot(ye_ref, ze_ref, 'ro', 'LineWidth', 2);
rectangle('Position', [rect_ycenter - rect_ly/2 - margin, rect_zcenter - rect_lz/2 - margin, rect_ly, rect_lz], ...
    'EdgeColor', [0.85 0.2 0.2], 'LineWidth', 1.5, 'LineStyle', '--');
hold off;
grid on;
axis equal;
legend('Trajectory', 'Target');
xlabel('y (m)');
ylabel('z (m)');
title('End-Effector Trajectory');

subplot(3,2,[5 6])
hold on;
stairs(time_log, tau1, 'b-', 'LineWidth', 2);
stairs(time_log, tau2, 'g-', 'LineWidth', 2);
hold off;
grid on;
legend('\tau_1', '\tau_2');
ylabel('Torque (N*m)');
xlabel('Time (s)');
title('Joint Torques');

sgtitle('Robotic Arm MPC - Design (Rectangular Constraint)');

function cineq = rectangularObstacleConstraint(X, ~, ~, ~, params, rectBounds)
% cineq <= 0 is feasible.
% This function constrains all predicted end-effector points (except current
% state) and interpolated points along each prediction segment to stay
% outside the rectangle.

theta1 = X(:,1);
theta2 = X(:,3);

a1 = params(3);
a2 = params(4);
h = params(7);

y = a1*cos(theta1) + a2*cos(theta1 + theta2);
z = h + a1*sin(theta1) + a2*sin(theta1 + theta2);

y_min = rectBounds(1);
y_max = rectBounds(2);
z_min = rectBounds(3);
z_max = rectBounds(4);

% Skip current state.
insideMarginY = min(y(2:end) - y_min, y_max - y(2:end));
insideMarginZ = min(z(2:end) - z_min, z_max - z(2:end));

% Positive only when point is strictly inside rectangle. (Ignore current state.)
insideDepthPoints = min(insideMarginY, insideMarginZ);

% Also constrain points sampled along each line segment between consecutive
% prediction nodes so trajectories cannot pass through the rectangle between
% nodes. In this case we start from the current state.
nSegmentSamples = 10;
insideDepthSegments = [];
if numel(y) > 1
    y1 = y(1:end-1);
    y2 = y(2:end);
    z1 = z(1:end-1);
    z2 = z(2:end);

    t = linspace(0, 1, nSegmentSamples + 2);
    t = t(2:end-1); % interior samples only (endpoints already constrained)

    ySeg = y1 + (y2 - y1) * t;
    zSeg = z1 + (z2 - z1) * t;

    insideMarginYSeg = min(ySeg - y_min, y_max - ySeg);
    insideMarginZSeg = min(zSeg - z_min, z_max - zSeg);
    insideDepthSegments = min(insideMarginYSeg, insideMarginZSeg);
end

cineq = [insideDepthPoints; insideDepthSegments(:)];
end
