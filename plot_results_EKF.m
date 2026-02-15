% Extract log data from SimulationOutput object "out"
theta1_log = out.logsout.get('theta1').Values;
theta2_log = out.logsout.get('theta2').Values;
omega1_log = out.logsout.get('omega1').Values;
omega2_log = out.logsout.get('omega2').Values;
xhat_log = out.logsout.get('xhat').Values;
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
xlabel('Time (s)');
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
xlabel('Time (s)');
ylabel('z (m)');
title('End-Effector z');

% (2,1) theta1 and theta2
subplot(3,2,3)
hold on;
plot(theta1_log.Time, theta1_log.Data * 180/pi, 'b-', 'LineWidth', 2);
plot(theta2_log.Time, theta2_log.Data * 180/pi, 'g-', 'LineWidth', 2);
plot(xhat_log.Time, xhat_log.Data(:,1) * 180/pi, 'm-', 'LineWidth', 2);
plot(xhat_log.Time, xhat_log.Data(:,3) * 180/pi, 'c-', 'LineWidth', 2);
hold off;
grid on;
legend('\theta_1', '\theta_2', '\theta_1 EKF', '\theta_2 EKF', 'Location', 'best');
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Joint Angles');

% (2,2) omega1 and omega2
subplot(3,2,4)
hold on;
plot(omega1_log.Time, omega1_log.Data, 'b-', 'LineWidth', 2);
plot(omega2_log.Time, omega2_log.Data, 'g-', 'LineWidth', 2);
plot(xhat_log.Time, xhat_log.Data(:,2), 'm-', 'LineWidth', 2);
plot(xhat_log.Time, xhat_log.Data(:,4), 'c-', 'LineWidth', 2);
hold off;
grid on;
legend('\omega_1', '\omega_2', '\omega_1 EKF', '\omega_2 EKF', 'Location', 'best');
xlabel('Time (s)');
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
xlabel('Time (s)');
ylabel('Torque (N*m)');
xlabel('Time (s)');
title('Joint Torques');

% (3,2) estimated torque disturbances
subplot(3,2,6)
hold on;
plot(xhat_log.Time, xhat_log.Data(:,5), 'm-', 'LineWidth', 2);
plot(xhat_log.Time, xhat_log.Data(:,6), 'c-', 'LineWidth', 2);
hold off;
grid on;
legend('\tau_{d1} EKF', '\tau_{d2} EKF', 'Location', 'best');
xlabel('Time (s)');
ylabel('Disturbance Torque (N*m)');
title('Estimated Torque Disturbances');

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

%% Animate results (real-time videos)

% Extract animation time vector from end-effector y_log
t_anim = y_log.Time;
num_steps = numel(t_anim);

% If there are fewer than 2 steps, nothing to animate
if num_steps < 2
    return;
end

% Estimate average time step for animation; use 0.01s fallback if needed
dt_anim = mean(diff(t_anim));
if ~isfinite(dt_anim) || dt_anim <= 0
    dt_anim = 0.01;
end

% Determine time axis limits for animation (span of the log)
t_lim = [t_anim(1), t_anim(end)];

% Gather all values for axis limit calculations (merge logs and refs)
y_all = [y_log.Data; ye_ref_log.Data];
z_all = [z_log.Data; ze_ref_log.Data];
theta_all_deg = [theta1_log.Data; theta2_log.Data; xhat_log.Data(:,1); xhat_log.Data(:,3)] * 180/pi;
omega_all = [omega1_log.Data; omega2_log.Data; xhat_log.Data(:,2); xhat_log.Data(:,4)];
tau_all = [tau1_log.Data; tau2_log.Data];
taud_all = [xhat_log.Data(:,5); xhat_log.Data(:,6)];

% Compute axis limits and padding for all animated signals
y_min = min(y_all); y_max = max(y_all); y_pad = max(1e-6, 0.05*(y_max - y_min));
z_min = min(z_all); z_max = max(z_all); z_pad = max(1e-6, 0.05*(z_max - z_min));
theta_min = min(theta_all_deg); theta_max = max(theta_all_deg); theta_pad = max(1e-6, 0.05*(theta_max - theta_min));
omega_min = min(omega_all); omega_max = max(omega_all); omega_pad = max(1e-6, 0.05*(omega_max - omega_min));
tau_min = min(tau_all); tau_max = max(tau_all); tau_pad = max(1e-6, 0.05*(tau_max - tau_min));
taud_min = min(taud_all); taud_max = max(taud_all); taud_pad = max(1e-6, 0.05*(taud_max - taud_min));

% Set axis limits for each subplot
y_lim = [y_min - y_pad, y_max + y_pad];
z_lim = [z_min - z_pad, z_max + z_pad];
theta_lim = [theta_min - theta_pad, theta_max + theta_pad];
omega_lim = [omega_min - omega_pad, omega_max + omega_pad];
tau_lim = [tau_min - tau_pad, tau_max + tau_pad];
taud_lim = [taud_min - taud_pad, taud_max + taud_pad];

% Axis limits for trajectory plot (include all targets)
traj_y_all = [y_log.Data; ref1(1); ref2(1); ref3(1)];
traj_z_all = [z_log.Data; ref1(2); ref2(2); ref3(2)];
traj_y_min = min(traj_y_all); traj_y_max = max(traj_y_all); traj_y_pad = max(1e-6, 0.05*(traj_y_max - traj_y_min));
traj_z_min = min(traj_z_all); traj_z_max = max(traj_z_all); traj_z_pad = max(1e-6, 0.05*(traj_z_max - traj_z_min));
traj_y_lim = [traj_y_min - traj_y_pad, traj_y_max + traj_y_pad];
traj_z_lim = [traj_z_min - traj_z_pad, traj_z_max + traj_z_pad];

%--------------------%
% Figure for animated time series (6 subplots)
fig_anim = figure('Units', 'normalized', 'OuterPosition', [0 0 1 1]);

% (1,1) y position
subplot(3,2,1)
hold on;
stairs(ye_ref_log.Time, ye_ref_log.Data, 'r--', 'LineWidth', 2); % reference trajectory
y_line = plot(nan, nan, 'b-', 'LineWidth', 2);                  % animated actual y
hold off;
grid on;
legend('y_{ref}', 'y', 'Location', 'bestoutside');
xlabel('Time (s)');
ylabel('y (m)');
title('End-Effector y');
xlim(t_lim);
ylim(y_lim);

% (1,2) z position
subplot(3,2,2)
hold on;
stairs(ze_ref_log.Time, ze_ref_log.Data, 'r--', 'LineWidth', 2); % reference trajectory
z_line = plot(nan, nan, 'b-', 'LineWidth', 2);                  % animated actual z
hold off;
grid on;
legend('z_{ref}', 'z', 'Location', 'bestoutside');
xlabel('Time (s)');
ylabel('z (m)');
title('End-Effector z');
xlim(t_lim);
ylim(z_lim);

% (2,1) joint angles (measured and EKF estimates)
subplot(3,2,3)
hold on;
theta1_line = plot(nan, nan, 'b-', 'LineWidth', 2);      % measured theta1
theta2_line = plot(nan, nan, 'g-', 'LineWidth', 2);      % measured theta2
theta1_hat_line = plot(nan, nan, 'm-', 'LineWidth', 2);  % EKF theta1
theta2_hat_line = plot(nan, nan, 'c-', 'LineWidth', 2);  % EKF theta2
hold off;
grid on;
legend('\theta_1', '\theta_2', '\theta_1 EKF', '\theta_2 EKF', 'Location', 'bestoutside');
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Joint Angles');
xlim(t_lim);
ylim(theta_lim);

% (2,2) joint velocities (measured and EKF estimates)
subplot(3,2,4)
hold on;
omega1_line = plot(nan, nan, 'b-', 'LineWidth', 2);      % measured omega1
omega2_line = plot(nan, nan, 'g-', 'LineWidth', 2);      % measured omega2
omega1_hat_line = plot(nan, nan, 'm-', 'LineWidth', 2);  % EKF omega1
omega2_hat_line = plot(nan, nan, 'c-', 'LineWidth', 2);  % EKF omega2
hold off;
grid on;
legend('\omega_1', '\omega_2', '\omega_1 EKF', '\omega_2 EKF', 'Location', 'bestoutside');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Joint Velocities');
xlim(t_lim);
ylim(omega_lim);

% (3,1) joint torques (from controller, animated)
subplot(3,2,5)
hold on;
tau1_line = stairs(nan, nan, 'b-', 'LineWidth', 2);   % animated torque 1
tau2_line = stairs(nan, nan, 'g-', 'LineWidth', 2);   % animated torque 2
hold off;
grid on;
legend('\tau_1', '\tau_2', 'Location', 'bestoutside');
xlabel('Time (s)');
ylabel('Torque (N*m)');
title('Joint Torques');
xlim(t_lim);
ylim(tau_lim);

% (3,2) EKF estimated torque disturbances
subplot(3,2,6)
hold on;
taud1_line = plot(nan, nan, 'm-', 'LineWidth', 2);  % EKF disturbance 1
taud2_line = plot(nan, nan, 'c-', 'LineWidth', 2);  % EKF disturbance 2
hold off;
grid on;
legend('\tau_{d1} EKF', '\tau_{d2} EKF', 'Location', 'bestoutside');
xlabel('Time (s)');
ylabel('Disturbance Torque (N*m)');
title('Estimated Torque Disturbances');
xlim(t_lim);
ylim(taud_lim);

% Title for the animation window
sgtitle('Robotic Arm MPC - Simulation in Simulink with Simscape Multibody (Animation)');

%--------------------%
% Figure for animated trajectory in y-z (end-effector path)
fig_traj = figure('Units', 'normalized', 'OuterPosition', [0 0 1 1]);
hold on;
traj_line = plot(nan, nan, 'b-', 'LineWidth', 2);          % trajectory so far
traj_point = plot(nan, nan, 'ko', 'LineWidth', 2);         % current end-effector position
plot(ref1(1), ref1(2), 'ro', 'LineWidth', 2);              % Target 1
plot(ref2(1), ref2(2), 'go', 'LineWidth', 2);              % Target 2
plot(ref3(1), ref3(2), 'mo', 'LineWidth', 2);              % Target 3
hold off;
grid on;
axis equal;
legend('Trajectory', 'Current', 'Target 1', 'Target 2', 'Target 3', 'Location', 'bestoutside');
xlabel('y (m)');
ylabel('z (m)');
title('End-Effector Trajectory (Animation)');
xlim(traj_y_lim);
ylim(traj_z_lim);

%--------------------%
% Setup video writers for saving animations to files
video1 = VideoWriter('results_EKF_animation.mp4', 'MPEG-4');
video2 = VideoWriter('trajectory_EKF_animation.mp4', 'MPEG-4');
open(video1);
open(video2);

% Interpolate torque signals to animation time base (stepwise - 'previous')
tau1_log_time = t_anim;
tau2_log_time = t_anim;
tau1_log_interpolated = interp1(tau1_log.Time, tau1_log.Data, tau1_log_time, 'previous');
tau2_log_interpolated = interp1(tau2_log.Time, tau2_log.Data, tau2_log_time, 'previous');

%--------------------%
% Main animation loop: update plots for each timestep and record frames
for k = 1:num_steps
    tk = t_anim(k); % current time

    % Find all samples up to the current time for each signal
    y_idx = y_log.Time <= tk;
    z_idx = z_log.Time <= tk;
    t1_idx = theta1_log.Time <= tk;
    t2_idx = theta2_log.Time <= tk;
    w1_idx = omega1_log.Time <= tk;
    w2_idx = omega2_log.Time <= tk;
    u1_idx = tau1_log_time <= tk;
    u2_idx = tau2_log_time <= tk;
    xhat_idx = xhat_log.Time <= tk;

    % Update animated lines for each subplot with data up to current time
    set(y_line, 'XData', y_log.Time(y_idx), 'YData', y_log.Data(y_idx));
    set(z_line, 'XData', z_log.Time(z_idx), 'YData', z_log.Data(z_idx));

    set(theta1_line, 'XData', theta1_log.Time(t1_idx), 'YData', theta1_log.Data(t1_idx) * 180/pi);
    set(theta2_line, 'XData', theta2_log.Time(t2_idx), 'YData', theta2_log.Data(t2_idx) * 180/pi);
    set(theta1_hat_line, 'XData', xhat_log.Time(xhat_idx), 'YData', xhat_log.Data(xhat_idx,1) * 180/pi);
    set(theta2_hat_line, 'XData', xhat_log.Time(xhat_idx), 'YData', xhat_log.Data(xhat_idx,3) * 180/pi);

    set(omega1_line, 'XData', omega1_log.Time(w1_idx), 'YData', omega1_log.Data(w1_idx));
    set(omega2_line, 'XData', omega2_log.Time(w2_idx), 'YData', omega2_log.Data(w2_idx));
    set(omega1_hat_line, 'XData', xhat_log.Time(xhat_idx), 'YData', xhat_log.Data(xhat_idx,2));
    set(omega2_hat_line, 'XData', xhat_log.Time(xhat_idx), 'YData', xhat_log.Data(xhat_idx,4));

    set(tau1_line, 'XData', tau1_log_time(u1_idx), 'YData', tau1_log_interpolated(u1_idx));
    set(tau2_line, 'XData', tau2_log_time(u2_idx), 'YData', tau2_log_interpolated(u2_idx));

    set(taud1_line, 'XData', xhat_log.Time(xhat_idx), 'YData', xhat_log.Data(xhat_idx,5));
    set(taud2_line, 'XData', xhat_log.Time(xhat_idx), 'YData', xhat_log.Data(xhat_idx,6));

    % Update the animated end-effector trajectory
    set(traj_line, 'XData', y_log.Data(y_idx), 'YData', z_log.Data(z_idx));
    % Update the point showing the current position on trajectory
    if any(y_idx)
        set(traj_point, 'XData', y_log.Data(find(y_idx, 1, 'last')), 'YData', z_log.Data(find(z_idx, 1, 'last')));
    end

    % Render the updates and save each frame to video files
    drawnow;
    writeVideo(video1, getframe(fig_anim));
    writeVideo(video2, getframe(fig_traj));

    % Pause (in real time) to pace the animation as in simulated time
    if k < num_steps
        pause(max(0, t_anim(k+1) - t_anim(k)));
    end
end

% Close and finalize video writers
close(video1);
close(video2);