%% 7-DOF Redundant Manipulator: Kinematics, IK, and Smooth Joint Trajectory
% Purpose:
%   Research-grade MATLAB script for a 7-DOF serial manipulator.
%   It computes two IK solutions (start and goal end-effector poses), then
%   generates a smooth joint-space trajectory between them.
%
% Robot model:
%   7-DOF revolute serial chain with Denavit-Hartenberg (DH) parameters.
%
% Algorithms used:
%   1) Forward kinematics via chained homogeneous transforms:
%        T(q) = A1(q1) A2(q2) ... A7(q7)
%   2) Redundant IK via Jacobian pseudoinverse with damped least squares and
%      null-space regularization for joint-centering:
%        Delta q = J^ e + (I - J^ J) z
%   3) Minimum-jerk (quintic) time-scaling for smooth motion with zero
%      endpoint velocity and acceleration.
%
% Inputs (defined in this script):
%   - Robot DH parameters
%   - Desired start and goal end-effector poses
%
% Outputs (workspace variables):
%   - q_start, q_goal: IK joint solutions
%   - t_vec: time vector
%   - q_traj, qd_traj, qdd_traj: joint position/velocity/acceleration profiles

clear;
clc;
close all;

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

%% Desired End-Effector Poses
% Pose is represented as T = [R p; 0 1].
% Start and goal are fully specified SE(3) targets.
p_des_start = [-0.5; 0; 1.5+sqrt(3)/2];
R_des_start = [0 0 1; 0 1 0; -1 0 0]';
T_des_start = [R_des_start, p_des_start; 0 0 0 1];

p_des_goal = [4.0; 0; -1.3];
R_des_goal = [-1 0 0; 0 1 0; 0 0 -1]';
T_des_goal = [R_des_goal, p_des_goal; 0 0 0 1];

%% Inverse Kinematics
% IK error definition in task space:
%   e = [e_p; e_or]
% where e_p = p_des - p_current and e_or is orientation error in axis-angle
% form from R_err = R_des * R_current'.
%
% Redundancy resolution:
%   Delta q = J^ e + (I - J^ J) z
% with z chosen to keep joints near the center of their limits.
ik_options = struct();
ik_options.maxIterations = 350;
ik_options.tolPosition = 1e-4;
ik_options.tolOrientation = 1e-4;
ik_options.stepSize = 0.65;
ik_options.damping = 1e-3;
ik_options.nullSpaceGain = 0.08;
ik_options.jointCenterGain = 1.0;
ik_options.nRandomRestarts = 20;
ik_options.display = true;

q_seed = zeros(7, 1);
[q_start, report_start] = inverseKinematics(T_des_start, q_seed, robot, ik_options);
[q_goal, report_goal] = inverseKinematics(T_des_goal, q_start, robot, ik_options);

fprintf('Start IK: converged=%d, iter=%d, |e_p|=%.3e, |e_o|=%.3e\n', ...
    report_start.converged, report_start.iterations, report_start.posErrorNorm, report_start.oriErrorNorm);
fprintf('Goal  IK: converged=%d, iter=%d, |e_p|=%.3e, |e_o|=%.3e\n', ...
    report_goal.converged, report_goal.iterations, report_goal.posErrorNorm, report_goal.oriErrorNorm);

if ~(report_start.converged && report_goal.converged)
    warning('At least one IK solve did not fully converge. Inspect reports before using the trajectory in closed-loop experiments.');
end

if norm(q_start - q_goal) < 1e-6
    error('q_start and q_goal are numerically identical. Change seeds or target poses.');
end

%% Trajectory Generation
% Minimum-jerk quintic trajectory for each joint:
%   q(t) = q0 + (qf - q0) * s(t)
%   s(t) = 10 tau^3 - 15 tau^4 + 6 tau^5,  tau = t / T
%
% This polynomial enforces:
%   s(0)=0, s(T)=1, s_dot(0)=s_dot(T)=0, s_ddot(0)=s_ddot(T)=0
% therefore position, velocity, and acceleration are smooth at endpoints.
traj_options = struct();
traj_options.totalTime = 12.0;
traj_options.numSamples = 241;

[t_vec, q_traj, qd_traj, qdd_traj] = trajectoryGeneration(q_start, q_goal, traj_options);

%% Forward Kinematics Validation
[T_start_ach, ~] = forwardKinematics(q_traj(:, 1), robot);
[T_goal_ach, ~] = forwardKinematics(q_traj(:, end), robot);

pos_err_start = norm(T_start_ach(1:3, 4) - p_des_start);
ori_err_start = rotationErrorAngle(R_des_start, T_start_ach(1:3, 1:3));
pos_err_goal = norm(T_goal_ach(1:3, 4) - p_des_goal);
ori_err_goal = rotationErrorAngle(R_des_goal, T_goal_ach(1:3, 1:3));

fprintf('Start pose error: position=%.3e m, orientation=%.3e rad\n', pos_err_start, ori_err_start);
fprintf('Goal  pose error: position=%.3e m, orientation=%.3e rad\n', pos_err_goal, ori_err_goal);

%% Simulation / Visualization
% Precompute end-effector path for plotting.
n_samples = size(q_traj, 2);
ee_path = zeros(3, n_samples);
for k = 1:n_samples
    T_k = forwardKinematics(q_traj(:, k), robot);
    ee_path(:, k) = T_k(1:3, 4);
end

figure('Color', 'w', 'Name', '7-DOF Redundant Manipulator Motion');
ax = axes('Projection', 'perspective');
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
view(ax, 36, 22);
xlabel(ax, 'X [m]');
ylabel(ax, 'Y [m]');
zlabel(ax, 'Z [m]');
title(ax, 'Resolved-Rate IK + Quintic Joint Trajectory');

plot3(ax, ee_path(1, :), ee_path(2, :), ee_path(3, :), 'k--', 'LineWidth', 1.1);
plot3(ax, p_des_start(1), p_des_start(2), p_des_start(3), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 7);
plot3(ax, p_des_goal(1), p_des_goal(2), p_des_goal(3), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 7);
quiver3(ax, p_des_start(1), p_des_start(2), p_des_start(3), ...
    R_des_start(1, 3), R_des_start(2, 3), R_des_start(3, 3), 0.5, ...
    'Color', [0 0 0.8], 'LineWidth', 1.4, 'MaxHeadSize', 0.8);
quiver3(ax, p_des_goal(1), p_des_goal(2), p_des_goal(3), ...
    R_des_goal(1, 3), R_des_goal(2, 3), R_des_goal(3, 3), 0.5, ...
    'Color', [0 0.5 0], 'LineWidth', 1.4, 'MaxHeadSize', 0.8);

[~, joints_0] = forwardKinematics(q_traj(:, 1), robot);
h_robot = plot3(ax, joints_0(1, :), joints_0(2, :), joints_0(3, :), '-o', ...
    'Color', [0.15 0.45 0.85], 'LineWidth', 2.0, 'MarkerSize', 4, 'MarkerFaceColor', [0.15 0.45 0.85]);

xlim(ax, [-3 6]);
ylim(ax, [-3 3]);
zlim(ax, [-3 4]);

for k = 1:n_samples
    [~, joints_k] = forwardKinematics(q_traj(:, k), robot);
    set(h_robot, 'XData', joints_k(1, :), 'YData', joints_k(2, :), 'ZData', joints_k(3, :));
    drawnow;
    pause(0.015);
end

%% Joint Profiles
figure('Color', 'w', 'Name', 'Joint Profiles');
tiledlayout(3, 1, 'TileSpacing', 'compact');

nexttile;
plot(t_vec, q_traj.', 'LineWidth', 1.2);
grid on;
ylabel('q [rad]');
title('Joint Position');
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7', 'Location', 'eastoutside');

nexttile;
plot(t_vec, qd_traj.', 'LineWidth', 1.2);
grid on;
ylabel('qdot [rad/s]');
title('Joint Velocity');

nexttile;
plot(t_vec, qdd_traj.', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('qddot [rad/s^2]');
title('Joint Acceleration');

%% Outputs in Workspace
% q_start, q_goal, t_vec, q_traj, qd_traj, qdd_traj, report_start, report_goal


function [T_current, joint_positions] = forwardKinematics(q, robot)
% FORWARDKINEMATICS Computes end-effector pose using chained DH transforms.
%
% Mathematical model:
%   T(q) = T_base * A1(q1) * A2(q2) * ... * A7(q7)
% where each Ai follows the standard DH convention.

n = robot.nJoints;
T_current = robot.baseT;
joint_positions = zeros(3, n + 1);
joint_positions(:, 1) = T_current(1:3, 4);

for i = 1:n
    A_i = dhTransform(q(i), robot.d(i), robot.a(i), robot.alpha(i));
    T_current = T_current * A_i;
    joint_positions(:, i + 1) = T_current(1:3, 4);
end

end


function [q_solution, report] = inverseKinematics(T_des, q_init, robot, options)
% INVERSEKINEMATICS Solves SE(3) IK with Jacobian pseudoinverse and null-space term.
%
% Orientation error is represented in axis-angle form by:
%   R_err = R_des * R_current'
%   e_or = log_SO3(R_err)
% and total task error is e = [e_p; e_or].

q_init = q_init(:);
q_center = mean(robot.jointLimits, 2);
best_error = inf;
best_report = struct();
q_solution = q_init;

for restart = 1:options.nRandomRestarts
    if restart == 1
        q_current = q_init;
    else
        q_current = robot.jointLimits(:, 1) + ...
            (robot.jointLimits(:, 2) - robot.jointLimits(:, 1)) .* rand(robot.nJoints, 1);
    end

    converged = false;
    pos_error_norm = inf;
    ori_error_norm = inf;

    for iter = 1:options.maxIterations
        T_current = forwardKinematics(q_current, robot);
        p_current = T_current(1:3, 4);
        R_current = T_current(1:3, 1:3);

        p_des = T_des(1:3, 4);
        R_des = T_des(1:3, 1:3);

        e_p = p_des - p_current;
        R_err = R_des * R_current';
        e_o = rotationMatrixToAxisAngle(R_err);
        pose_error = [e_p; e_o];

        pos_error_norm = norm(e_p);
        ori_error_norm = norm(e_o);
        if pos_error_norm < options.tolPosition && ori_error_norm < options.tolOrientation
            converged = true;
            break;
        end

        J = computeJacobian(q_current, robot);

        % Damped least-squares pseudoinverse for robustness near singularity.
        lambda = options.damping;
        J_pinv = J' / (J * J' + (lambda^2) * eye(6));

        % Null-space bias that pulls the solution away from joint limits.
        z = -options.jointCenterGain * (q_current - q_center);
        delta_q = J_pinv * pose_error + (eye(robot.nJoints) - J_pinv * J) * (options.nullSpaceGain * z);

        q_current = q_current + options.stepSize * delta_q;
        q_current = max(min(q_current, robot.jointLimits(:, 2)), robot.jointLimits(:, 1));
    end

    total_error = pos_error_norm + ori_error_norm;
    if total_error < best_error
        best_error = total_error;
        q_solution = q_current;
        best_report.converged = converged;
        best_report.iterations = iter;
        best_report.posErrorNorm = pos_error_norm;
        best_report.oriErrorNorm = ori_error_norm;
    end

    if converged
        break;
    end
end

report = best_report;
if options.display
    fprintf('IK solved with residual |e_p|=%.3e, |e_o|=%.3e\n', report.posErrorNorm, report.oriErrorNorm);
end

end


function J = computeJacobian(q, robot)
% COMPUTEJACOBIAN Numerical geometric Jacobian in base frame.
%
% For small Delta q_i, the Jacobian column is approximated by finite difference:
%   Jv_i ~= (p(q + eps*e_i) - p(q - eps*e_i)) / (2*eps)
%   Jw_i from skew( Rdot * R' ), where Rdot uses central differences.

n = robot.nJoints;
eps_fd = 1e-6;
J = zeros(6, n);

[T0, ~] = forwardKinematics(q, robot);
R0 = T0(1:3, 1:3);

for i = 1:n
    q_plus = q;
    q_minus = q;
    q_plus(i) = q_plus(i) + eps_fd;
    q_minus(i) = q_minus(i) - eps_fd;

    T_plus = forwardKinematics(q_plus, robot);
    T_minus = forwardKinematics(q_minus, robot);

    p_plus = T_plus(1:3, 4);
    p_minus = T_minus(1:3, 4);
    J(1:3, i) = (p_plus - p_minus) / (2 * eps_fd);

    R_plus = T_plus(1:3, 1:3);
    R_minus = T_minus(1:3, 1:3);
    R_dot = (R_plus - R_minus) / (2 * eps_fd);
    Omega = R_dot * R0';
    Omega = 0.5 * (Omega - Omega');
    J(4:6, i) = [Omega(3, 2); Omega(1, 3); Omega(2, 1)];
end

end


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


function A = dhTransform(theta, d, a, alpha)
% DHTRANSFORM Returns standard DH homogeneous transform.
ct = cos(theta);
st = sin(theta);
ca = cos(alpha);
sa = sin(alpha);

A = [ct, -st * ca,  st * sa, a * ct; ...
     st,  ct * ca, -ct * sa, a * st; ...
      0,       sa,       ca,      d; ...
      0,        0,        0,      1];

end


function rotvec = rotationMatrixToAxisAngle(R)
% ROTATIONMATRIXTOAXISANGLE Maps R in SO(3) to axis-angle vector in R^3.
cos_theta = (trace(R) - 1) / 2;
cos_theta = max(-1, min(1, cos_theta));
theta = acos(cos_theta);

if theta < 1e-9
    S = 0.5 * (R - R');
    rotvec = [S(3, 2); S(1, 3); S(2, 1)];
    return;
end

if abs(pi - theta) < 1e-5
    A = (R + eye(3)) / 2;
    axis = [sqrt(max(A(1, 1), 0)); sqrt(max(A(2, 2), 0)); sqrt(max(A(3, 3), 0))];
    if R(3, 2) - R(2, 3) < 0
        axis(1) = -axis(1);
    end
    if R(1, 3) - R(3, 1) < 0
        axis(2) = -axis(2);
    end
    if R(2, 1) - R(1, 2) < 0
        axis(3) = -axis(3);
    end
    axis_norm = norm(axis);
    if axis_norm < 1e-12
        axis = [1; 0; 0];
    else
        axis = axis / axis_norm;
    end
    rotvec = theta * axis;
    return;
end

S = (R - R') / (2 * sin(theta));
axis = [S(3, 2); S(1, 3); S(2, 1)];
rotvec = theta * axis;

end


function err = rotationErrorAngle(R_des, R_current)
% ROTATIONERRORANGLE Scalar orientation error angle in radians.
R_err = R_des * R_current';
rotvec = rotationMatrixToAxisAngle(R_err);
err = norm(rotvec);

end
