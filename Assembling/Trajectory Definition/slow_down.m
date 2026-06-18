T_new = 12*5;
numSamples = 241;

traj_options_slow = struct();
traj_options_slow.totalTime = T_new;
traj_options_slow.numSamples = numSamples*5;

t_vec_slow = [];
q_traj_slow = [];
qd_traj_slow = [];
qdd_traj_slow = [];

t_offset = 0;

segment_length = numSamples - 1;   % 240

for i_seg = 1:41

    idx_start = (i_seg-1)*segment_length + 1;
    idx_end   = idx_start + segment_length;

    q_start = q_traj(:, idx_start);
    q_goal  = q_traj(:, idx_end);

    [t_seg, q_seg, qd_seg, qdd_seg] = trajectoryGeneration( ...
        q_start, q_goal, traj_options_slow);

    if i_seg > 1
        t_seg   = t_seg(2:end);
        q_seg   = q_seg(:,2:end);
        qd_seg  = qd_seg(:,2:end);
        qdd_seg = qdd_seg(:,2:end);
    end

    t_vec_slow = [t_vec_slow, t_seg + t_offset];
    q_traj_slow = [q_traj_slow, q_seg];
    qd_traj_slow = [qd_traj_slow, qd_seg];
    qdd_traj_slow = [qdd_traj_slow, qdd_seg];

    t_offset = t_vec_slow(end);
end


%% PLOT
% In this context, discontinuity means an unexpected sample-to-sample spike
% in joint position/velocity/acceleration. For redundant robots, moderate
% joint variations are normal; only values above clear thresholds are flagged.

joint_labels = {'q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6', 'q_7'};
colors = lines(7);

% Figure 1: Joint positions (all 7 joints)
figure('Color', 'w', 'Name', 'Joint Positions', 'NumberTitle', 'off');
hold on;
for j = 1:7
    plot(t_vec_slow, q_traj_slow(j, :), 'LineWidth', 1.4, 'Color', colors(j, :), 'DisplayName', joint_labels{j});
end
grid on;
xlabel('Time [s]');
ylabel('q [rad]');
title('Joint Positions q(t)');
legend('Location', 'eastoutside');
xlim([t_vec_slow(1), t_vec_slow(end)]);

% Figure 2: Joint velocities (all 7 joints)
figure('Color', 'w', 'Name', 'Joint Velocities', 'NumberTitle', 'off');
hold on;
for j = 1:7
    plot(t_vec_slow, qd_traj_slow(j, :), 'LineWidth', 1.4, 'Color', colors(j, :), 'DisplayName', joint_labels{j});
end
grid on;
xlabel('Time [s]');
ylabel('qdot [rad/s]');
title('Joint Velocities qdot(t)');
legend('Location', 'eastoutside');
xlim([t_vec_slow(1), t_vec_slow(end)]);

% Figure 3: Joint accelerations (all 7 joints)
figure('Color', 'w', 'Name', 'Joint Accelerations', 'NumberTitle', 'off');
hold on;
for j = 1:7
    plot(t_vec_slow, qdd_traj_slow(j, :), 'LineWidth', 1.4, 'Color', colors(j, :), 'DisplayName', joint_labels{j});
end
grid on;
xlabel('Time [s]');
ylabel('qddot [rad/s^2]');
title('Joint Accelerations qddot(t)');
legend('Location', 'eastoutside');
xlim([t_vec_slow(1), t_vec_slow(end)]);

% Discontinuity metrics at sample level.
delta_q = diff(q_traj_slow, 1, 2);
delta_qd = diff(qd_traj_slow, 1, 2);
delta_qdd = diff(qdd_traj_slow, 1, 2);

max_pos_jump = max(abs(delta_q), [], 2);
max_vel_jump = max(abs(delta_qd), [], 2);
max_acc_jump = max(abs(delta_qdd), [], 2);

global_max_pos = max(max_pos_jump);
global_max_vel = max(max_vel_jump);
global_max_acc = max(max_acc_jump);

% Thresholds for significant discontinuity detection.
pos_thresh = 1e-2;
vel_thresh = 1e-1;
acc_thresh = 1;

has_pos_disc = any(max_pos_jump > pos_thresh);
has_vel_disc = any(max_vel_jump > vel_thresh);
has_acc_disc = any(max_acc_jump > acc_thresh);

if has_pos_disc || has_vel_disc || has_acc_disc
    fprintf('Warning: discontinuities detected in trajectory. Max |Delta q|=%.3e, Max |Delta qdot|=%.3e, Max |Delta qddot|=%.3e\n', ...
        global_max_pos, global_max_vel, global_max_acc);
else
    fprintf('Trajectory is smooth: no significant discontinuities detected. Max |Delta q|=%.3e, Max |Delta qdot|=%.3e, Max |Delta qddot|=%.3e\n', ...
        global_max_pos, global_max_vel, global_max_acc);
end


%% FCT

function [t, q, qd, qdd] = trajectoryGeneration(q_start, q_goal, options)
% TRAJECTORYGENERATION Minimum-jerk quintic interpolation in joint space.
%
% q(t) = q_start + (q_goal - q_start) s(t), with
% s(t) = 10 tau^3 - 15 tau^4 + 6 tau^5, tau = t/T.

T = options.totalTime;
n_samples = options.numSamples;

t = linspace(0, T, n_samples);
tau = t / T;

s = 10 * tau.^3 - 15 * tau.^4 + 6 * tau.^5;
s_dot = (30 * tau.^2 - 60 * tau.^3 + 30 * tau.^4) / T;
s_ddot = (60 * tau - 180 * tau.^2 + 120 * tau.^3) / (T^2);

delta_q = q_goal - q_start;
q = q_start + delta_q * s;
qd = delta_q * s_dot;
qdd = delta_q * s_ddot;

end