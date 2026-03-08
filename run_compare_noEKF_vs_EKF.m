%% Run no-EKF and EKF simulations, then plot comparison
% 1) init, init_mpc, run control.slx -> store output
% 2) clear workspace except that output
% 3) init, init_mpc_EKF, run control_EKF.slx -> store output
% 4) Plot comparison: theta1, theta2, torques; 2D trajectory with targets

%% ----- Run no-EKF (control.slx) -----
init;
init_mpc;
out_full_state = sim('control');

%% ----- Keep only no-EKF output, then run EKF -----
clearvars -except out_full_state;

init;
init_mpc_EKF;
out_EKF = sim('control_EKF');

%% ----- Extract logged signals for comparison -----
% Full State
theta1_noEKF = out_full_state.logsout.get('theta1').Values;
theta2_noEKF = out_full_state.logsout.get('theta2').Values;
tau1_noEKF  = out_full_state.logsout.get('tau1').Values;
tau2_noEKF  = out_full_state.logsout.get('tau2').Values;
y_noEKF     = out_full_state.logsout.get('y').Values;
z_noEKF     = out_full_state.logsout.get('z').Values;

% EKF
theta1_EKF = out_EKF.logsout.get('theta1').Values;
theta2_EKF = out_EKF.logsout.get('theta2').Values;
tau1_EKF   = out_EKF.logsout.get('tau1').Values;
tau2_EKF   = out_EKF.logsout.get('tau2').Values;
y_EKF      = out_EKF.logsout.get('y').Values;
z_EKF      = out_EKF.logsout.get('z').Values;

% Target points
ref1 = [0.2, 0.3];
ref2 = [0.2, 0.6];
ref3 = [-0.2, 0.6];

%% ----- Figure 1: theta1, theta2, torques (Full State vs EKF) -----
figure('Name', 'Comparison: Joint Positions and Torques - Measured Full State vs EKF');
sgtitle('Comparison: Joint Positions and Torques - Measured Full State vs EKF');

subplot(3,1,1)
hold on;
plot(theta1_noEKF.Time, theta1_noEKF.Data * 180/pi, 'b-', 'LineWidth', 2);
plot(theta1_EKF.Time,   theta1_EKF.Data * 180/pi, 'g-', 'LineWidth', 2);
hold off;
grid on;
legend('\theta_1 (Full State)', '\theta_1 (EKF)', 'Location', 'best');
xlabel('Time (s)');
ylabel('Angle (deg)');
title('\theta_1');

subplot(3,1,2)
hold on;
plot(theta2_noEKF.Time, theta2_noEKF.Data * 180/pi, 'b-', 'LineWidth', 2);
plot(theta2_EKF.Time,   theta2_EKF.Data * 180/pi, 'g-', 'LineWidth', 2);
hold off;
grid on;
legend('\theta_2 (Full State)', '\theta_2 (EKF)', 'Location', 'best');
xlabel('Time (s)');
ylabel('Angle (deg)');
title('\theta_2');

subplot(3,1,3)
hold on;
stairs(tau1_noEKF.Time, tau1_noEKF.Data, 'b-', 'LineWidth', 2);
stairs(tau2_noEKF.Time, tau2_noEKF.Data, 'c-', 'LineWidth', 2);
stairs(tau1_EKF.Time,   tau1_EKF.Data, 'g-', 'LineWidth', 2);
stairs(tau2_EKF.Time,   tau2_EKF.Data, 'r-', 'LineWidth', 2);
hold off;
grid on;
legend('\tau_1 (Full State)', '\tau_2 (Full State)', '\tau_1 (EKF)', '\tau_2 (EKF)', 'Location', 'best');
xlabel('Time (s)');
ylabel('Torque (N\cdotm)');
title('Joint Torques');

%% ----- Figure 2: 2D end-effector trajectory (Full State vs EKF) + 3 targets -----
figure('Name', 'Trajectory Comparison: Full State vs EKF');
hold on;
plot(y_noEKF.Data, z_noEKF.Data, 'b-', 'LineWidth', 2);
plot(y_EKF.Data,   z_EKF.Data,   'g-', 'LineWidth', 2);
plot(ref1(1), ref1(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot(ref2(1), ref2(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(ref3(1), ref3(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
hold off;
grid on;
axis equal;
legend('End-effector (Full State)', 'End-effector (EKF)', 'Target 1', 'Target 2', 'Target 3', 'Location', 'best');
xlabel('y (m)');
ylabel('z (m)');
title('Trajectory Comparison: Full State vs EKF');
