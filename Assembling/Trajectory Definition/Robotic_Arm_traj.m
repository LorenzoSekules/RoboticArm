function [t_vec, q_traj, qd_traj, qdd_traj] = Robotic_Arm_traj(varargin)

%% 7-DOF Redundant Manipulator: Kinematics, IK, and Smooth Joint Trajectory
% Purpose:
%   Research-grade MATLAB script for a 7-DOF serial manipulator.
%   It computes IK solutions for one or more end-effector waypoints, then
%   generates a smooth joint-space trajectory across all waypoints.
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
% Inputs:
%   Backward-compatible (two-pose mode):
%     Robotic_Arm_traj(p_start, R_start, p_goal, R_goal)
%   Multi-waypoint mode:
%     Robotic_Arm_traj(T_waypoints)                      with size 4x4xN
%     Robotic_Arm_traj(p_waypoints, R_waypoints)         with size 3xN and 3x3xN
%
% Outputs (workspace variables):
%   - q_start, q_goal: first/last IK joint solutions
%   - q_waypoints: IK joint solutions at all waypoints
%   - t_vec: time vector
%   - q_traj, qd_traj, qdd_traj: joint position/velocity/acceleration profiles

% Parse inputs while keeping backward compatibility.
T_waypoints = parseWaypointInputs(varargin{:});
n_waypoints = size(T_waypoints, 3);
if n_waypoints < 2
    error('At least two waypoints are required to generate a trajectory.');
end

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
robot.d = [2, 0, 2, 0, 2, 0, 1.15];
robot.baseT = [0 0 1 0; 0 1 0 0; -1 0 0 0; 0 0 0 1];
robot.jointLimits = repmat([-pi, pi], 7, 1);

%% Desired End-Effector Waypoints
% Pose is represented as T = [R p; 0 1].
% Waypoints are fully specified SE(3) targets.
T_des_start = T_waypoints(:, :, 1);
T_des_goal = T_waypoints(:, :, end);
p_des_start = T_des_start(1:3, 4);
R_des_start = T_des_start(1:3, 1:3);
p_des_goal = T_des_goal(1:3, 4);
R_des_goal = T_des_goal(1:3, 1:3);

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

% The key continuity rule for redundant robots:
%   q_i = IK(T_i, q_{i-1})
% 
%   In a redundant manipulator, the same Cartesian pose may admit multiple
%   joint configurations (different branches). Solving each segment independently
%   can switch branches at shared waypoints, creating non-physical jumps in q.
%   Using the previous joint solution as the next seed keeps IK on the local,
%   nearest branch and therefore preserves physical continuity.
q_waypoints = zeros(robot.nJoints, n_waypoints);
report_waypoints(1, n_waypoints) = struct('converged', false, 'iterations', 0, ...
    'posErrorNorm', inf, 'oriErrorNorm', inf, 'transitionEnergy', inf);

% --- Minimum-energy trajectory selection across multiple IK branches ---
% Redundant manipulators can realize the same Cartesian waypoints with
% different joint-space branches. We generate several waypoint-consistent
% trajectories (always seeded as q_i = IK(T_i, q_{i-1})) and select the one
% with least joint motion energy proxy.
%
% Energy proxy used here:
%   E = sum_k ||q(:,k+1) - q(:,k)||^2
% Minimizing E reduces unnecessary joint displacement, which generally
% correlates with lower actuation effort and smoother physical motion.
nTrajectoryCandidates = 2;
% Large additive penalty used to keep unconstrained optimization structure
% while strongly discouraging configurations that collide.
collisionPenaltyWeight = 1e5;
best_energy = inf;
best_total_residual = inf;
best_all_converged = false;
best_q_waypoints = q_waypoints;
best_report_waypoints = report_waypoints;

for traj_idx = 1:nTrajectoryCandidates
    q_waypoints_candidate = zeros(robot.nJoints, n_waypoints);
    report_candidate(1, n_waypoints) = struct('converged', false, 'iterations', 0, ...
        'posErrorNorm', inf, 'oriErrorNorm', inf, 'transitionEnergy', inf);

    % Different initial seeds allow exploration of different valid IK branches.
    if traj_idx == 1
        q_seed_0 = zeros(robot.nJoints, 1);
    else
        q_seed_0 = robot.jointLimits(:, 1) + ...
            (robot.jointLimits(:, 2) - robot.jointLimits(:, 1)) .* rand(robot.nJoints, 1);
    end

    ik_local = ik_options;
    ik_local.display = false;

    for i_wp = 1:n_waypoints
        if i_wp == 1
            q_prev = q_seed_0;
        else
            q_prev = q_waypoints_candidate(:, i_wp - 1);
        end

        [q_i, report_i] = inverseKinematics(T_waypoints(:, :, i_wp), q_prev, robot, ik_local);
        q_waypoints_candidate(:, i_wp) = q_i;
        report_candidate(i_wp) = report_i;
    end

    delta_q_wp = diff(q_waypoints_candidate, 1, 2);
    energy_candidate = sum(sum(delta_q_wp.^2, 1));

    % Collision-aware augmentation for unconstrained selection:
    % we keep the same energy objective and add a large penalty whenever a
    % waypoint configuration violates floor/obstacle/self-collision checks.
    % This preserves the original architecture and only changes scoring.
    n_collision_waypoints = 0;
    for i_wp = 1:n_waypoints
        if check_collisions_arm(q_waypoints_candidate(:, i_wp))
            n_collision_waypoints = n_collision_waypoints + 1;
        end
    end
    energy_candidate = energy_candidate + collisionPenaltyWeight * n_collision_waypoints;

    total_residual_candidate = sum([report_candidate.posErrorNorm]) + sum([report_candidate.oriErrorNorm]);
    all_converged_candidate = all([report_candidate.converged]);

    % Selection criterion:
    % 1) Prefer fully converged trajectories.
    % 2) Among equally converged sets, choose minimum energy.
    % 3) Tie-break by total residual.
    if all_converged_candidate && ~best_all_converged
        best_all_converged = true;
        best_energy = energy_candidate;
        best_total_residual = total_residual_candidate;
        best_q_waypoints = q_waypoints_candidate;
        best_report_waypoints = report_candidate;
    elseif all_converged_candidate == best_all_converged
        if energy_candidate < best_energy || ...
                (abs(energy_candidate - best_energy) < 1e-12 && total_residual_candidate < best_total_residual)
            best_energy = energy_candidate;
            best_total_residual = total_residual_candidate;
            best_q_waypoints = q_waypoints_candidate;
            best_report_waypoints = report_candidate;
        end
    end
end

q_waypoints = best_q_waypoints;
report_waypoints = best_report_waypoints;

if ~all([report_waypoints.converged])
    warning('Selected minimum-energy trajectory has at least one non-converged waypoint IK solution.');
end

q_start = q_waypoints(:, 1);
q_goal = q_waypoints(:, end);
report_start = report_waypoints(1);
report_goal = report_waypoints(end);

fprintf('Start IK: converged=%d, iter=%d, |e_p|=%.3e, |e_o|=%.3e\n', ...
    report_start.converged, report_start.iterations, report_start.posErrorNorm, report_start.oriErrorNorm);
fprintf('Goal  IK: converged=%d, iter=%d, |e_p|=%.3e, |e_o|=%.3e\n', ...
    report_goal.converged, report_goal.iterations, report_goal.posErrorNorm, report_goal.oriErrorNorm);

if any(~[report_waypoints.converged])
    warning('At least one waypoint IK solve did not fully converge. Inspect report_waypoints before closed-loop use.');
end

if norm(q_start - q_goal) < 1e-6 && n_waypoints == 2
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

% Generate one continuous trajectory over all waypoints.
% The interpolation mathematics is unchanged: each segment uses the same
% minimum-jerk quintic profile already implemented in trajectoryGeneration.
n_segments = n_waypoints - 1;
t_vec = [];
q_traj = [];
qd_traj = [];
qdd_traj = [];
t_offset = 0;

for i_seg = 1:n_segments
    [t_seg, q_seg, qd_seg, qdd_seg] = trajectoryGeneration(q_waypoints(:, i_seg), q_waypoints(:, i_seg + 1), traj_options);

    if i_seg > 1
        % Remove duplicated boundary sample so the assembled path is continuous.
        t_seg = t_seg(2:end);
        q_seg = q_seg(:, 2:end);
        qd_seg = qd_seg(:, 2:end);
        qdd_seg = qdd_seg(:, 2:end);
    end

    t_vec = [t_vec, t_seg + t_offset]; %#ok<AGROW>
    q_traj = [q_traj, q_seg]; %#ok<AGROW>
    qd_traj = [qd_traj, qd_seg]; %#ok<AGROW>
    qdd_traj = [qdd_traj, qdd_seg]; %#ok<AGROW>
    t_offset = t_vec(end);
end

%% Forward Kinematics Validation
[T_start_ach, ~] = forwardKinematics(q_traj(:, 1), robot);
[T_goal_ach, ~] = forwardKinematics(q_traj(:, end), robot);

pos_err_start = norm(T_start_ach(1:3, 4) - p_des_start);
ori_err_start = rotationErrorAngle(R_des_start, T_start_ach(1:3, 1:3));
pos_err_goal = norm(T_goal_ach(1:3, 4) - p_des_goal);
ori_err_goal = rotationErrorAngle(R_des_goal, T_goal_ach(1:3, 1:3));

fprintf('Start pose error: position=%.3e m, orientation=%.3e rad\n', pos_err_start, ori_err_start);
fprintf('Goal  pose error: position=%.3e m, orientation=%.3e rad\n', pos_err_goal, ori_err_goal);

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
% 
% %% Joint Profiles
% figure('Color', 'w', 'Name', 'Joint Profiles');
% tiledlayout(3, 1, 'TileSpacing', 'compact');
% 
% nexttile;
% plot(t_vec, q_traj.', 'LineWidth', 1.2);
% grid on;
% ylabel('q [rad]');
% title('Joint Position');
% legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7', 'Location', 'eastoutside');
% 
% nexttile;
% plot(t_vec, qd_traj.', 'LineWidth', 1.2);
% grid on;
% ylabel('qdot [rad/s]');
% title('Joint Velocity');
% 
% nexttile;
% plot(t_vec, qdd_traj.', 'LineWidth', 1.2);
% grid on;
% xlabel('Time [s]');
% ylabel('qddot [rad/s^2]');
% title('Joint Acceleration');

%% Outputs in Workspace
% q_start, q_goal, t_vec, q_traj, qd_traj, qdd_traj, report_start, report_goal


function T_waypoints = parseWaypointInputs(varargin)
% PARSEWAYPOINTINPUTS Supports both the original two-pose API and waypoint APIs.

if nargin == 4
    p_des_start = varargin{1};
    R_des_start = varargin{2};
    p_des_goal = varargin{3};
    R_des_goal = varargin{4};

    validateattributes(p_des_start, {'numeric'}, {'size', [3, 1]}, mfilename, 'p_des_start');
    validateattributes(R_des_start, {'numeric'}, {'size', [3, 3]}, mfilename, 'R_des_start');
    validateattributes(p_des_goal, {'numeric'}, {'size', [3, 1]}, mfilename, 'p_des_goal');
    validateattributes(R_des_goal, {'numeric'}, {'size', [3, 3]}, mfilename, 'R_des_goal');

    T_waypoints = zeros(4, 4, 2);
    T_waypoints(:, :, 1) = [R_des_start, p_des_start; 0 0 0 1];
    T_waypoints(:, :, 2) = [R_des_goal, p_des_goal; 0 0 0 1];
    return;
end

if nargin == 1
    T_waypoints = varargin{1};
    validateattributes(T_waypoints, {'numeric'}, {'ndims', 3, 'size', [4, 4, NaN]}, mfilename, 'T_waypoints');
    if size(T_waypoints, 3) < 2
        error('T_waypoints must contain at least two transforms.');
    end
    return;
end

if nargin == 2
    p_waypoints = varargin{1};
    R_waypoints = varargin{2};
    validateattributes(p_waypoints, {'numeric'}, {'2d', 'nrows', 3}, mfilename, 'p_waypoints');
    validateattributes(R_waypoints, {'numeric'}, {'ndims', 3}, mfilename, 'R_waypoints');

    n_wp = size(p_waypoints, 2);
    if size(R_waypoints, 1) ~= 3 || size(R_waypoints, 2) ~= 3 || size(R_waypoints, 3) ~= n_wp
        error('R_waypoints must be 3x3xN and match p_waypoints size 3xN.');
    end
    if n_wp < 2
        error('At least two waypoints are required.');
    end

    T_waypoints = zeros(4, 4, n_wp);
    for k = 1:n_wp
        T_waypoints(:, :, k) = [R_waypoints(:, :, k), p_waypoints(:, k); 0 0 0 1];
    end
    return;
end

error(['Unsupported input format. Use either ', ...
    'Robotic_Arm_traj(p_start,R_start,p_goal,R_goal), ', ...
    'Robotic_Arm_traj(T_waypoints), or Robotic_Arm_traj(p_waypoints,R_waypoints).']);

end


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
%
% Candidate selection policy (minimal-energy branch choice):
%   Among all restart candidates that satisfy tolerances, choose the one
%   minimizing transition energy from q_init, approximated by
%   ||q_candidate - q_init||^2. This prefers the closest configuration
%   branch while preserving the existing IK update law.

q_init = q_init(:);
q_center = mean(robot.jointLimits, 2);
best_error = inf;
best_transition_energy = inf;
best_has_converged = false;
best_report = struct('converged', false, 'iterations', 0, ...
    'posErrorNorm', inf, 'oriErrorNorm', inf, 'transitionEnergy', inf);
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
    transition_energy = norm(q_current - q_init)^2;

    if converged
        % Prefer converged candidates with minimum transition energy.
        if ~best_has_converged || transition_energy < best_transition_energy || ...
                (abs(transition_energy - best_transition_energy) < 1e-12 && total_error < best_error)
            best_has_converged = true;
            best_transition_energy = transition_energy;
            best_error = total_error;
            q_solution = q_current;
            best_report.converged = converged;
            best_report.iterations = iter;
            best_report.posErrorNorm = pos_error_norm;
            best_report.oriErrorNorm = ori_error_norm;
            best_report.transitionEnergy = transition_energy;
        end
    else
        % If nothing converges, keep the smallest residual fallback.
        if ~best_has_converged && total_error < best_error
            best_error = total_error;
            best_transition_energy = transition_energy;
            q_solution = q_current;
            best_report.converged = converged;
            best_report.iterations = iter;
            best_report.posErrorNorm = pos_error_norm;
            best_report.oriErrorNorm = ori_error_norm;
            best_report.transitionEnergy = transition_energy;
        end
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


end