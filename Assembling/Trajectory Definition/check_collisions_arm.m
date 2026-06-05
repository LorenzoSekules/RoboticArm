function isCollision = check_collisions_arm(q)
% CHECK_COLLISIONS_ARM Returns true when a 7-DOF arm configuration collides.
%
% Collision model implemented (simplified and modular):
% 1) Floor constraint:             z >= -1.5 for all relevant points
% 2) Central obstacle (AABB box):  x in [-1,1], y in [-1,1], z in [-1.5,1.5]
% 3) Self-collision approximation: link centers as points with minimum distance
% 4) End-effector as a point:      explicit floor/box checks on EE position
%
% Input:
%   q : 7x1 (or 1x7) joint vector in radians
%
% Output:
%   isCollision : logical true if any collision/violation is detected

% Force column-vector format and validate dimensions early.
q = q(:);
if numel(q) ~= 7
    error('check_collisions_arm expects a 7-element joint vector q.');
end

% Robot model is defined locally so this function is fully standalone and
% can be called from cost functions, nonlinear constraints, or diagnostics.
robot = getDefaultRobotModel();

% Compute base + joint positions from forward kinematics.
% joint_positions is 3x(7+1): column 1 is base, column 8 is end-effector.
[~, joint_positions] = forwardKinematicsLocal(q, robot);

% Approximate each link as its geometric center between adjacent joints.
% For 7 links we get 7 center points in 3D.
link_centers = 0.5 * (joint_positions(:, 1:end-1) + joint_positions(:, 2:end));

% Environment constants used by all checks.
floor_z = -2.81;
box_x = [-3, 1];
box_y = [-3, 1];
box_z = [-3, 2];

% -------------------------------------------------------------------------
% 1) Floor collision: reject if any joint point or link center is below floor.
% -------------------------------------------------------------------------
if any(joint_positions(3, :) < floor_z) || any(link_centers(3, :) < floor_z)
    isCollision = true;
    return;
end

% -------------------------------------------------------------------------
% 2) Obstacle collision: reject if any point lies inside the central box.
% -------------------------------------------------------------------------
all_points = [joint_positions, link_centers];
inside_x = all_points(1, :) >= box_x(1) & all_points(1, :) <= box_x(2);
inside_y = all_points(2, :) >= box_y(1) & all_points(2, :) <= box_y(2);
inside_z = all_points(3, :) >= box_z(1) & all_points(3, :) <= box_z(2);
if any(inside_x & inside_y & inside_z)
    isCollision = true;
    return;
end

% -------------------------------------------------------------------------
% 3) Self-collision: minimum distance among link-center points.
% Consecutive and non-consecutive pairs are both checked with configurable
% thresholds (non-consecutive kept stricter because they should stay apart).
% -------------------------------------------------------------------------
minDistConsecutive = 0.25;
minDistNonConsecutive = 0.35;
n_links = size(link_centers, 2);

for i = 1:n_links-1
    for j = i+1:n_links
        d_ij = norm(link_centers(:, i) - link_centers(:, j));

        if j == i + 1
            if d_ij < minDistConsecutive
                isCollision = true;
                return;
            end
        else
            if d_ij < minDistNonConsecutive
                isCollision = true;
                return;
            end
        end
    end
end

% -------------------------------------------------------------------------
% 4) End-effector point collision (explicit for clarity and future extension).
% -------------------------------------------------------------------------
ee = joint_positions(:, end);

if ee(3) < floor_z
    isCollision = true;
    return;
end

ee_inside_box = ee(1) >= box_x(1) && ee(1) <= box_x(2) && ...
                ee(2) >= box_y(1) && ee(2) <= box_y(2) && ...
                ee(3) >= box_z(1) && ee(3) <= box_z(2);
if ee_inside_box
    isCollision = true;
    return;
end

% If all checks pass, configuration is collision-free under this model.
isCollision = false;

end


function robot = getDefaultRobotModel()
% GETDEFAULTROBOTMODEL Returns the same 7-DOF DH model used in trajectory code.
robot = struct();
robot.nJoints = 7;
robot.alpha = [-pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, 0];
robot.a = zeros(1, 7);
robot.d = [2, 0, 2, 0, 2, 0, 1];
robot.baseT = [0 0 1 0; 0 1 0 0; -1 0 0 0; 0 0 0 1];
end


function [T_current, joint_positions] = forwardKinematicsLocal(q, robot)
% FORWARDKINEMATICSLOCAL Computes base-to-EE transform and all joint positions.

n = robot.nJoints;
T_current = robot.baseT;
joint_positions = zeros(3, n + 1);
joint_positions(:, 1) = T_current(1:3, 4);

for i = 1:n
    A_i = dhTransformLocal(q(i), robot.d(i), robot.a(i), robot.alpha(i));
    T_current = T_current * A_i;
    joint_positions(:, i + 1) = T_current(1:3, 4);
end

end


function A = dhTransformLocal(theta, d, a, alpha)
% DHTRANSFORMLOCAL Returns standard DH homogeneous transform.

ct = cos(theta);
st = sin(theta);
ca = cos(alpha);
sa = sin(alpha);

A = [ct, -st * ca,  st * sa, a * ct; ...
     st,  ct * ca, -ct * sa, a * st; ...
      0,       sa,       ca,      d; ...
      0,        0,        0,      1];

end
