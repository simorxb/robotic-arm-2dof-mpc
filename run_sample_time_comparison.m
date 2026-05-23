%% Compare MPC performance for three controller sample times
% 1) init, init_mpc (builds nlobj and control.slx)
% 2) Loop: set nlobj.Ts / horizons, sim('control'), store logsout signals
% 3) 2x2 figure: y, z, joint angles, torques (three Ts overlaid)

%% ----- Setup -----
init;
init_mpc;

model = 'control';

configs = {
    struct('name', 'Ts = 0.1',  'Ts', 0.1,  'PredictionHorizon', 20, 'ControlHorizon', 15);
    struct('name', 'Ts = 0.05', 'Ts', 0.05, 'PredictionHorizon', 20, 'ControlHorizon', 15);
    struct('name', 'Ts = 0.02', 'Ts', 0.02, 'PredictionHorizon', 20, 'ControlHorizon', 15);
};

results = cell(length(configs), 1);

%% ----- Simulate each sample time -----
for s = 1:length(configs)
    config = configs{s};

    nlobj.Ts = config.Ts;
    nlobj.PredictionHorizon = config.PredictionHorizon;
    nlobj.ControlHorizon = config.ControlHorizon;

    out = sim(model);

    results{s} = struct( ...
        'name', config.name, ...
        'theta1', out.logsout.get('theta1').Values, ...
        'theta2', out.logsout.get('theta2').Values, ...
        'omega1', out.logsout.get('omega1').Values, ...
        'omega2', out.logsout.get('omega2').Values, ...
        'y', out.logsout.get('y').Values, ...
        'z', out.logsout.get('z').Values, ...
        'tau1', out.logsout.get('tau1').Values, ...
        'tau2', out.logsout.get('tau2').Values, ...
        'ye_ref', out.logsout.get('ye_ref').Values, ...
        'ze_ref', out.logsout.get('ze_ref').Values);
end

%% ----- Comparison figure (2x2) -----
colors = {'b', 'g', 'c'};

figure('Name', 'MPC Sample Time Comparison');
sgtitle('MPC Sample Time Comparison (P = 20, C = 15)');

% (1,1) y vs y_ref
subplot(2, 2, 1)
hold on;
for s = 1:length(results)
    r = results{s};
    plot(r.y.Time, r.y.Data, [colors{s} '-'], 'LineWidth', 2, ...
        'DisplayName', ['y (' r.name ')']);
end
r0 = results{1};
stairs(r0.ye_ref.Time, r0.ye_ref.Data, 'r--', 'LineWidth', 2, 'DisplayName', 'y_{ref}');
hold off;
grid on;
legend('Location', 'best');
xlabel('Time (s)');
ylabel('y (m)');
title('End-Effector y');

% (1,2) z vs z_ref
subplot(2, 2, 2)
hold on;
for s = 1:length(results)
    r = results{s};
    plot(r.z.Time, r.z.Data, [colors{s} '-'], 'LineWidth', 2, ...
        'DisplayName', ['z (' r.name ')']);
end
stairs(r0.ze_ref.Time, r0.ze_ref.Data, 'r--', 'LineWidth', 2, 'DisplayName', 'z_{ref}');
hold off;
grid on;
legend('Location', 'best');
xlabel('Time (s)');
ylabel('z (m)');
title('End-Effector z');

% (2,1) theta1 and theta2
subplot(2, 2, 3)
hold on;
for s = 1:length(results)
    r = results{s};
    plot(r.theta1.Time, r.theta1.Data * 180/pi, [colors{s} '-'], 'LineWidth', 2, ...
        'DisplayName', ['\theta_1 (' r.name ')']);
    plot(r.theta2.Time, r.theta2.Data * 180/pi, [colors{s} '--'], 'LineWidth', 1.5, ...
        'DisplayName', ['\theta_2 (' r.name ')']);
end
hold off;
grid on;
legend('Location', 'best');
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Joint Angles');

% (2,2) tau1 and tau2
subplot(2, 2, 4)
hold on;
for s = 1:length(results)
    r = results{s};
    stairs(r.tau1.Time, r.tau1.Data, [colors{s} '-'], 'LineWidth', 2, ...
        'DisplayName', ['\tau_1 (' r.name ')']);
    stairs(r.tau2.Time, r.tau2.Data, [colors{s} '--'], 'LineWidth', 1.5, ...
        'DisplayName', ['\tau_2 (' r.name ')']);
end
hold off;
grid on;
legend('Location', 'best');
xlabel('Time (s)');
ylabel('Torque (N*m)');
title('Joint Torques');
