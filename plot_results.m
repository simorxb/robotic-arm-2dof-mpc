% Extract log data from SimulationOutput object "out"
theta1_log = out.logsout.get('theta1').Values;
theta2_log = out.logsout.get('theta2').Values;
omega1_log = out.logsout.get('omega1').Values;
omega2_log = out.logsout.get('omega2').Values;
y_log = out.logsout.get('y').Values;
z_log = out.logsout.get('z').Values;
tau1_log = out.logsout.get('tau1').Values;
tau2_log = out.logsout.get('tau2').Values;
ye_ref_log = out.logsout.get('ye_ref').Values;
ze_ref_log = out.logsout.get('ze_ref').Values;

% Plot results in Simulink
figure;

% (1,1) y vs y_ref
subplot(3,2,1)
hold on;
plot(y_log.Time, y_log.Data, 'b-', 'LineWidth', 2);
stairs(ye_ref_log.Time, ye_ref_log.Data, 'r--', 'LineWidth', 2);
hold off;
grid on;
legend('y', 'y_{ref}', 'Location', 'best');
ylabel('y (m)');
title('End-Effector y');

% (1,2) z vs z_ref
subplot(3,2,2)
hold on;
plot(z_log.Time, z_log.Data, 'b-', 'LineWidth', 2);
stairs(ze_ref_log.Time, ze_ref_log.Data, 'r--', 'LineWidth', 2);
hold off;
grid on;
legend('z', 'z_{ref}', 'Location', 'best');
ylabel('z (m)');
title('End-Effector z');

% (2,1) theta1 and theta2
subplot(3,2,3)
hold on;
plot(theta1_log.Time, theta1_log.Data * 180/pi, 'b-', 'LineWidth', 2);
plot(theta2_log.Time, theta2_log.Data * 180/pi, 'g-', 'LineWidth', 2);
hold off;
grid on;
legend('\theta_1', '\theta_2', 'Location', 'best');
ylabel('Angle (deg)');
title('Joint Angles');

% (2,2) omega1 and omega2
subplot(3,2,4)
hold on;
plot(omega1_log.Time, omega1_log.Data, 'b-', 'LineWidth', 2);
plot(omega2_log.Time, omega2_log.Data, 'g-', 'LineWidth', 2);
hold off;
grid on;
legend('\omega_1', '\omega_2', 'Location', 'best');
ylabel('Angular Velocity (rad/s)');
title('Joint Velocities');

% (3,1) tau1 and tau2
subplot(3,2,5)
hold on;
stairs(tau1_log.Time, tau1_log.Data, 'b-', 'LineWidth', 2);
stairs(tau2_log.Time, tau2_log.Data, 'g-', 'LineWidth', 2);
hold off;
grid on;
legend('\tau_1', '\tau_2', 'Location', 'best');
ylabel('Torque (N*m)');
xlabel('Time (s)');
title('Joint Torques');

sgtitle('Robotic Arm MPC - Simulation in Simulink with Simscape Multibody');

% Plot end effector trajectory
figure;
ref1 = [0.2, 0.3];
ref2 = [0.2, 0.6];
ref3 = [-0.2, 0.6];
hold on;
plot(y_log.Data, z_log.Data, 'b-', 'LineWidth', 2);
plot(ref1(1), ref1(2), 'ro', 'LineWidth', 2);
plot(ref2(1), ref2(2), 'go', 'LineWidth', 2);
plot(ref3(1), ref3(2), 'mo', 'LineWidth', 2);
hold off;
grid on;
axis equal;
legend('Trajectory', 'Target 1', 'Target 2', 'Target 3');
xlabel('y (m)');
ylabel('z (m)');
title('End-Effector Trajectory');