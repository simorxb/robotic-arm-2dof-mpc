%% Nonlinear Model Predictive Controller (NMPC) Configuration for Robotic Arm

model = 'control';

% Number of states (theta1, omega1, theta2, omega2)
nx = 4;
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
% Physical parameters vector: [m1; m2; a1; a2; kj1; kj2]
% Values assumed initialized elsewhere
m1 = 0.49 + 0.59;
m2 = 0.37 + 0.18;
params = [m1; m2; a1; a2; kj1; kj2; lbz-rj1];

% Controller sample time (s)
nlobj.Ts = 0.1;

% Prediction and control horizons
nlobj.PredictionHorizon = 15;
nlobj.ControlHorizon = 4;

% Assign state and output functions
nlobj.Model.StateFcn = "stateFcnRoboticArm";
nlobj.Model.OutputFcn = "outputFcnRoboticArm";

% Input (torque) constraints
nlobj.ManipulatedVariables(1).Max = 10;  % Max torque joint 1 (N*m)
nlobj.ManipulatedVariables(1).Min = -10; % Min torque joint 1 (N*m)
nlobj.ManipulatedVariables(2).Max = 10;  % Max torque joint 2 (N*m)
nlobj.ManipulatedVariables(2).Min = -10; % Min torque joint 2 (N*m)

nlobj.ManipulatedVariables(1).ScaleFactor = 2;
nlobj.ManipulatedVariables(2).ScaleFactor = 2;

nlobj.OutputVariables(1).ScaleFactor = 1; % End-effector position x (m)
nlobj.OutputVariables(2).ScaleFactor = 1; % End-effector position y (m)

% Weights for cost function (tracking end-effector, penalize large torques & rate)
nlobj.Weights.OutputVariables = [1 1];
nlobj.Weights.ManipulatedVariables = [0.0 0.0];
nlobj.Weights.ManipulatedVariablesRate = [0.01 0.01];

% Initial conditions for validation
x0 = [0 0 0 0];        % [theta1; omega1; theta2; omega2]
u0 = [0 0];            % [tau1; tau2]
validateFcns(nlobj, x0, u0, [], {params});

%% Simulate MPC for Robotic Arm

% Initial states and references for the robotic arm
x0 = [0 0 0 0];           % [theta1; omega1; theta2; omega2]
u0 = [0 0];               % [tau1; tau2]
 
% Reference: end-effector position (x, y)
theta1_ref = pi/4;
theta2_ref = pi/2;
xe_ref = a1*cos(theta1_ref) + a2*cos(theta1_ref + theta2_ref);
ye_ref = lbz - rj1 + a1*sin(theta1_ref) + a2*sin(theta1_ref + theta2_ref);
ref = [xe_ref ye_ref];

Tend = 1;
steps = round(Tend/nlobj.Ts);

nloptions = nlmpcmoveopt;
nloptions.Parameters = {params};

x_log = zeros(steps, nx);
u_log = zeros(steps, nu);
time_log = zeros(steps, 1);

for i = 1:steps
    x_log(i,:) = x0;
    time_log(i) = (i-1)*nlobj.Ts;
    [mv,nloptions] = nlmpcmove(nlobj, x0, u0, ref, [], nloptions);
    u_log(i,:) = mv';
    
    x0 = nloptions.X0(1,:);
    u0 = mv';

    % Print percentage of completion
    percent_done = (i / steps) * 100;
    fprintf('Simulation progress: %.1f%%\n', percent_done);
end

%% Plot results
theta1 = x_log(:,1);
theta2 = x_log(:,3);
x_eff = a1*cos(theta1) + a2*cos(theta1 + theta2);
y_eff = lbz - rj1 + a1*sin(theta1) + a2*sin(theta1 + theta2);

tau1 = u_log(:,1);
tau2 = u_log(:,2);

figure;

subplot(3,2,1)
hold on;
stairs(time_log, x_eff, 'b-', 'LineWidth', 2);
stairs(time_log, ref(1)*ones(size(x_eff)), 'r--', 'LineWidth', 2);
hold off;
grid on;
legend('x', 'x_{ref}');
ylabel('Position (m)');
title('End-Effector x');

subplot(3,2,2)
hold on;
stairs(time_log, y_eff, 'g-', 'LineWidth', 2);
stairs(time_log, ref(2)*ones(size(y_eff)), 'm--', 'LineWidth', 2);
hold off;
grid on;
legend('y', 'y_{ref}');
ylabel('Position (m)');
title('End-Effector y');

subplot(3,2,3)
hold on;
stairs(time_log, theta1*180/pi, 'b-', 'LineWidth', 2);
stairs(time_log, theta2*180/pi, 'g-', 'LineWidth', 2);
hold off;
grid on;
legend('\theta_1', '\theta_2');
ylabel('Angle (deg)');
title('Joint Angles');

subplot(3,2,4)
plot(x_eff, y_eff, 'k-', 'LineWidth', 2);
hold on;
plot(ref(1), ref(2), 'ro', 'LineWidth', 2);
hold off;
grid on;
axis equal;
legend('Trajectory', 'Target');
xlabel('x (m)');
ylabel('y (m)');
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

sgtitle('Robotic Arm MPC Simulation');
