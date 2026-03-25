% Main launcher for robust redundant-DOF trajectory generation.
% Run this file to execute the full pipeline:
% 1) redundant IK branch search (same EE pose),
% 2) smooth trajectory generation,
% 3) forbidden-box checking and null-space repair,
% 4) animation with show().

clear; clc;

%% Robot Parameters
% Standard DH matrix for link i:
%   Ai = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)
%
% The homogeneous transform encodes both orientation and position:
%   T = [R p;
%        0 1]
% where R in SO(3), p in R^3.
robot = struct();
robot.nJoints = 7;
robot.alpha = [-pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, 0];
robot.a = zeros(1, 7);
robot.d = [2, 0, 2, 0, 2, 0, 1];
robot.baseT = [0 0 1 0; 0 1 0 0; -1 0 0 0; 0 0 0 1];
robot.jointLimits = repmat([-pi, pi], 7, 1);

%% Trejectory
% Pose is represented as T = [R p; 0 1].
% Start and goal are fully specified SE(3) targets.

% Build a single waypoint sequence and solve it in one call.
% This is compatible with the updated Robotic_Arm_traj interface and keeps
% joint-space continuity internally across all A->B->C->... transitions.
R_up = [0 0 1; 0 1 0; -1 0 0]';
R_down = [-1 0 0; 0 1 0; 0 0 -1]';

% Named waypoints (columns of p_waypoints) for readability.
% Each pX is [x; y; z] in meters.
p1  = [-0.5;               0.0;  1.5 + sqrt(3)/2];
p2  = [ 4.0;               0.0; -1.3];
p3  = [-0.7;               0.0;  1.5 + sqrt(3)/2];
p4  = [ 4.0 + sqrt(3);     0.0; -1.3];
p5  = [-0.9;               0.0;  1.5 + sqrt(3)/2];
p6  = [ 4.0 + sqrt(3)/2;  -1.5; -1.3];
p7  = [-1.1;               0.0;  1.5 + sqrt(3)/2];
p8  = [ 4.0 - sqrt(3)/2;  -1.5; -1.3];
p9  = [-1.3;               0.0;  1.5 + sqrt(3)/2];
p10 = [ 4.0 - sqrt(3);     0.0; -1.3];
p11 = [-1.5;               0.0;  1.5 + sqrt(3)/2];
p12 = [ 4.0 - sqrt(3)/2;   1.5; -1.3];
p13 = [-1.7;               0.0;  1.5 + sqrt(3)/2];
p14 = [ 4.0 + sqrt(3)/2;   1.5; -1.3];

% Waypoint order: p1 -> p2 -> ... -> p14
p_waypoints = [p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14];

n_wp = size(p_waypoints, 2);
R_waypoints = zeros(3, 3, n_wp);
for k = 1:n_wp
    if mod(k, 2) == 1
        R_waypoints(:, :, k) = R_up;
    else
        R_waypoints(:, :, k) = R_down;
    end
end

[t_vec, q_traj, qd_traj, qdd_traj] = Robotic_Arm_traj(p_waypoints, R_waypoints);

%% Trajectory Analysis and Continuity Check
% In this context, discontinuity means an unexpected sample-to-sample spike
% in joint position/velocity/acceleration. For redundant robots, moderate
% joint variations are normal; only values above clear thresholds are flagged.

joint_labels = {'q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6', 'q_7'};
colors = lines(7);

% Figure 1: Joint positions (all 7 joints)
figure('Color', 'w', 'Name', 'Joint Positions', 'NumberTitle', 'off');
hold on;
for j = 1:7
    plot(t_vec, q_traj(j, :), 'LineWidth', 1.4, 'Color', colors(j, :), 'DisplayName', joint_labels{j});
end
grid on;
xlabel('Time [s]');
ylabel('q [rad]');
title('Joint Positions q(t)');
legend('Location', 'eastoutside');
xlim([t_vec(1), t_vec(end)]);

% Figure 2: Joint velocities (all 7 joints)
figure('Color', 'w', 'Name', 'Joint Velocities', 'NumberTitle', 'off');
hold on;
for j = 1:7
    plot(t_vec, qd_traj(j, :), 'LineWidth', 1.4, 'Color', colors(j, :), 'DisplayName', joint_labels{j});
end
grid on;
xlabel('Time [s]');
ylabel('qdot [rad/s]');
title('Joint Velocities qdot(t)');
legend('Location', 'eastoutside');
xlim([t_vec(1), t_vec(end)]);

% Figure 3: Joint accelerations (all 7 joints)
figure('Color', 'w', 'Name', 'Joint Accelerations', 'NumberTitle', 'off');
hold on;
for j = 1:7
    plot(t_vec, qdd_traj(j, :), 'LineWidth', 1.4, 'Color', colors(j, :), 'DisplayName', joint_labels{j});
end
grid on;
xlabel('Time [s]');
ylabel('qddot [rad/s^2]');
title('Joint Accelerations qddot(t)');
legend('Location', 'eastoutside');
xlim([t_vec(1), t_vec(end)]);

% Discontinuity metrics at sample level.
delta_q = diff(q_traj, 1, 2);
delta_qd = diff(qd_traj, 1, 2);
delta_qdd = diff(qdd_traj, 1, 2);

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

% %% Simulation / Visualization
% % Precompute end-effector path for plotting.
% n_samples = size(q_traj, 2);
% ee_path = zeros(3, n_samples);
% for k = 1:n_samples
%     T_k = forwardKinematics(q_traj(:, k), robot);
%     ee_path(:, k) = T_k(1:3, 4);
% end
% 
% figure('Color', 'w', 'Name', '7-DOF Redundant Manipulator Motion');
% ax = axes('Projection', 'perspective');
% hold(ax, 'on');
% grid(ax, 'on');
% axis(ax, 'equal');
% view(ax, 36, 22);
% xlabel(ax, 'X [m]');
% ylabel(ax, 'Y [m]');
% zlabel(ax, 'Z [m]');
% title(ax, 'Resolved-Rate IK + Quintic Joint Trajectory');
% 
% plot3(ax, ee_path(1, :), ee_path(2, :), ee_path(3, :), 'k--', 'LineWidth', 1.1);
% plot3(ax, p_des_start(1), p_des_start(2), p_des_start(3), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 7);
% plot3(ax, p_des_goal(1), p_des_goal(2), p_des_goal(3), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 7);
% quiver3(ax, p_des_start(1), p_des_start(2), p_des_start(3), ...
%     R_des_start(1, 3), R_des_start(2, 3), R_des_start(3, 3), 0.5, ...
%     'Color', [0 0 0.8], 'LineWidth', 1.4, 'MaxHeadSize', 0.8);
% quiver3(ax, p_des_goal(1), p_des_goal(2), p_des_goal(3), ...
%     R_des_goal(1, 3), R_des_goal(2, 3), R_des_goal(3, 3), 0.5, ...
%     'Color', [0 0.5 0], 'LineWidth', 1.4, 'MaxHeadSize', 0.8);
% 
% [~, joints_0] = forwardKinematics(q_traj(:, 1), robot);
% h_robot = plot3(ax, joints_0(1, :), joints_0(2, :), joints_0(3, :), '-o', ...
%     'Color', [0.15 0.45 0.85], 'LineWidth', 2.0, 'MarkerSize', 4, 'MarkerFaceColor', [0.15 0.45 0.85]);
% 
% xlim(ax, [-3 6]);
% ylim(ax, [-3 3]);
% zlim(ax, [-3 4]);
% 
% for k = 1:n_samples
%     [~, joints_k] = forwardKinematics(q_traj(:, k), robot);
%     set(h_robot, 'XData', joints_k(1, :), 'YData', joints_k(2, :), 'ZData', joints_k(3, :));
%     drawnow;
%     pause(0.015);
% end

%%
K_tile_force = 2000/0.015;
C_tile_force = 2*5*sqrt(K_tile_force*2.598*0.2*100);

K_tile_torque = 250/deg2rad(10);
C_tile_torque = 2*1*sqrt(K_tile_torque*0.541266*100)*0.7;

