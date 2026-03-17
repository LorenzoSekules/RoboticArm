function [t, qTraj, info] = plan_7dof_constrained_trajectory(qStart, qGoal, L, opts)

if nargin < 4
    opts = struct();
end

opts = set_defaults(opts);

qStart = qStart(:);
qGoal = qGoal(:);

if numel(qStart) ~= 7 || numel(qGoal) ~= 7
    error('qStart and qGoal must be 7x1 vectors.');
end

N = opts.numWaypoints;
if N < 3
    error('numWaypoints must be at least 3.');
end

t = linspace(0, opts.totalTime, N).';
s = smoothstep5((t - t(1)) / (t(end) - t(1)));
qRef = qStart.' + s * (qGoal - qStart).';

x0 = reshape(qRef(2:end-1, :).', [], 1);
lb = -pi * ones(size(x0));
ub = pi * ones(size(x0));

hasFmincon = exist('fmincon', 'file') == 2;

if ~hasFmincon
    warning('Optimization Toolbox (fmincon) not found. Returning quintic interpolation trajectory.');
    qTraj = qRef;
    info = struct('exitflag', -999, 'fval', NaN, 'maxConstraint', NaN, 'message', 'fmincon not available');
    return
end

solverOpts = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...
    'Display', opts.display, ...
    'MaxFunctionEvaluations', opts.maxFunctionEvaluations, ...
    'MaxIterations', opts.maxIterations, ...
    'StepTolerance', 1e-9, ...
    'ConstraintTolerance', 1e-4, ...
    'OptimalityTolerance', 1e-4);

obj = @(x) objective_fun(x, qRef, opts);
nonl = @(x) nonlinear_constraints(x, qStart, qGoal, L, opts);

[xSol, fval, exitflag, output] = fmincon(obj, x0, [], [], [], [], lb, ub, nonl, solverOpts);

qTraj = unpack_path(xSol, qStart, qGoal);
[c, ~] = nonlinear_constraints(xSol, qStart, qGoal, L, opts);

info = struct();
info.exitflag = exitflag;
info.fval = fval;
info.maxConstraint = max(c);
info.output = output;

end


function opts = set_defaults(opts)

if ~isfield(opts, 'numWaypoints'); opts.numWaypoints = 61; end
if ~isfield(opts, 'totalTime'); opts.totalTime = 20; end
if ~isfield(opts, 'boxHalfSize'); opts.boxHalfSize = [1; 1; 1.5]; end
if ~isfield(opts, 'boxMargin'); opts.boxMargin = 0.05; end
if ~isfield(opts, 'boxIgnoreFirstJoints'); opts.boxIgnoreFirstJoints = 1; end
if ~isfield(opts, 'boxCheckSamples'); opts.boxCheckSamples = 21; end
if ~isfield(opts, 'selfCollisionMinDist'); opts.selfCollisionMinDist = 0.15; end
if ~isfield(opts, 'selfCollisionSlackNeighbors'); opts.selfCollisionSlackNeighbors = 1; end
if ~isfield(opts, 'linkSamples'); opts.linkSamples = 4; end
if ~isfield(opts, 'display'); opts.display = 'iter'; end
if ~isfield(opts, 'maxIterations'); opts.maxIterations = 180; end
if ~isfield(opts, 'maxFunctionEvaluations'); opts.maxFunctionEvaluations = 120000; end

if numel(opts.boxHalfSize) ~= 3
    error('boxHalfSize must be a 3x1 vector [bx; by; bz].');
end

opts.boxHalfSize = opts.boxHalfSize(:);

end


function f = objective_fun(x, qRef, ~)

Q = [qRef(1, :); reshape(x(:), 7, []).'; qRef(end, :)];

D1 = diff(Q, 1, 1);
D2 = diff(Q, 2, 1);

termTrack = sum((Q(:) - qRef(:)).^2);
termVel = sum(D1(:).^2);
termSmooth = sum(D2(:).^2);

wTrack = 2.0;
wVel = 0.5;
wSmooth = 12.0;

f = wTrack * termTrack + wVel * termVel + wSmooth * termSmooth;

end


function [c, ceq] = nonlinear_constraints(x, qStart, qGoal, L, opts)

Q = unpack_path(x, qStart, qGoal);
numKnots = size(Q, 1);

c = [];

for k = 1:numKnots
    qk = Q(k, :).';

    [~, segA, segB] = robot_geometry(qk, L);

    % Forbidden box: every checked link segment must stay out of the inflated box.
    for i = 1:size(segA, 2)
        if i <= opts.boxIgnoreFirstJoints
            continue
        end
        dSeg = segment_box_signed_distance(segA(:, i), segB(:, i), opts.boxHalfSize, opts.boxMargin, opts.boxCheckSamples);
        c(end + 1, 1) = -dSeg; %#ok<AGROW>
    end

    % Self collision: non-neighboring links keep minimum separation.
    numSeg = size(segA, 2);
    inactiveDist = 1e6;
    for i = 1:numSeg
        for j = i + 1:numSeg
            if abs(i - j) <= opts.selfCollisionSlackNeighbors
                d = inactiveDist;
            elseif norm(segB(:, i) - segA(:, i)) < 1e-8
                d = inactiveDist;
            elseif norm(segB(:, j) - segA(:, j)) < 1e-8
                d = inactiveDist;
            elseif norm(segB(:, i) - segA(:, j)) < 1e-8
                d = inactiveDist;
            else
                d = segment_segment_distance(segA(:, i), segB(:, i), segA(:, j), segB(:, j));
            end
            c(end + 1, 1) = opts.selfCollisionMinDist - d; %#ok<AGROW>
        end
    end
end

ceq = [];

end


function Q = unpack_path(x, qStart, qGoal)

x = x(:);
Qmid = reshape(x, 7, []).';
Q = [qStart.'; Qmid; qGoal.'];

end


function s = smoothstep5(u)

u = min(max(u, 0), 1);
s = 10 * u.^3 - 15 * u.^4 + 6 * u.^5;

end


function [P, segA, segB] = robot_geometry(q, L)

alpha = [-pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, 0];
d = [L(1), L(2), L(3), L(4), L(5), L(6), L(7)];
a = zeros(1, 7);

T = eye(4) * [0 0 1 0; 0 1 0 0; -1 0 0 0; 0 0 0 1];
P = zeros(3, 8);
P(:, 1) = [0; 0; 0];

for i = 1:7
    ct = cos(q(i));
    st = sin(q(i));
    ca = cos(alpha(i));
    sa = sin(alpha(i));

    A = [ct, -st * ca, st * sa, a(i) * ct; ...
         st, ct * ca, -ct * sa, a(i) * st; ...
         0, sa, ca, d(i); ...
         0, 0, 0, 1];

    T = T * A;
    P(:, i + 1) = T(1:3, 4);
end

segA = P(:, 1:end-1);
segB = P(:, 2:end);

end


function d = segment_box_signed_distance(aPt, bPt, halfSize, margin, nSamples)

halfInflated = halfSize + margin;

d = inf;
for k = 0:nSamples
    t = k / nSamples;
    p = (1 - t) * aPt + t * bPt;
    d = min(d, sdf_box(p, halfInflated));
end

end


function d = sdf_box(p, halfSize)

q = abs(p) - halfSize;
outside = max(q, 0);
outsideDist = norm(outside);
insideDist = min(max(q), 0);
d = outsideDist + insideDist;

end


function d = segment_segment_distance(p1, q1, p2, q2)

u = q1 - p1;
v = q2 - p2;
w0 = p1 - p2;

a = dot(u, u);
b = dot(u, v);
c = dot(v, v);
dd = dot(u, w0);
e = dot(v, w0);

den = a * c - b * b;
small = 1e-12;

if den < small
    sN = 0;
    sD = 1;
    tN = e;
    tD = c;
else
    sN = (b * e - c * dd);
    tN = (a * e - b * dd);
    sD = den;
    tD = den;

    if sN < 0
        sN = 0;
        tN = e;
        tD = c;
    elseif sN > sD
        sN = sD;
        tN = e + b;
        tD = c;
    end
end

if tN < 0
    tN = 0;
    if -dd < 0
        sN = 0;
    elseif -dd > a
        sN = sD;
    else
        sN = -dd;
        sD = a;
    end
elseif tN > tD
    tN = tD;
    if (-dd + b) < 0
        sN = 0;
    elseif (-dd + b) > a
        sN = sD;
    else
        sN = (-dd + b);
        sD = a;
    end
end

if abs(sN) < small
    sc = 0;
else
    sc = sN / sD;
end

if abs(tN) < small
    tc = 0;
else
    tc = tN / tD;
end

dP = w0 + sc * u - tc * v;
d = norm(dP);

end
