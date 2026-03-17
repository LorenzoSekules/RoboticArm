function [qFeasible, report] = constrained_ik_7dof(Tdes, qSeed, L, opts)

if nargin < 5
    opts = struct();
end

opts = set_defaults(opts);

qSeed = qSeed(:);

if ~all(size(Tdes) == [4, 4])
    error('Invalid Tdes size. Expect 4x4 homogeneous transform.');
end

pDes = Tdes(1:3, 4);
RDes = Tdes(1:3, 1:3);

if numel(qSeed) ~= 7
    error('Invalid input size. Expect qSeed 7x1.');
end

hasFmincon = exist('fmincon', 'file') == 2;
if ~hasFmincon
    error('constrained_ik_7dof requires fmincon (Optimization Toolbox).');
end

lb = -pi * ones(7, 1);
ub = pi * ones(7, 1);
q0 = min(max(qSeed, lb), ub);

solverOpts = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...
    'Display', opts.display, ...
    'MaxFunctionEvaluations', opts.maxFunctionEvaluations, ...
    'MaxIterations', opts.maxIterations, ...
    'StepTolerance', 1e-10, ...
    'ConstraintTolerance', opts.ikTol, ...
    'OptimalityTolerance', 1e-5);

obj  = @(q) ik_objective(q, qSeed, opts);
nonl = @(q) ik_constraints(q, pDes, RDes, L, opts);

solverQuiet = optimoptions(solverOpts, 'Display', 'off');

% Pre-screen random candidates: keep those with lowest forbidden-box violation.
nRandCandidates = max(opts.multiStartN * 30, 300);
qCandMat        = lb + rand(7, nRandCandidates) .* (ub - lb);
boxViolVec      = zeros(1, nRandCandidates);
for kk = 1:nRandCandidates
    [~, sA, sB] = robot_geometry(qCandMat(:, kk), L);
    vk = 0;
    for ii = (opts.boxIgnoreFirstJoints + 1):size(sA, 2)
        dS = segment_box_signed_distance(sA(:, ii), sB(:, ii), opts.boxHalfSize, opts.boxMargin, opts.boxCheckSamples);
        vk = max(vk, -min(dS, 0));
    end
    boxViolVec(kk) = vk;
end
[~, sortIdx]   = sort(boxViolVec);
prescreenSeeds = qCandMat(:, sortIdx(1:opts.multiStartN - 1));
allSeeds       = [q0, prescreenSeeds];

bestSol      = q0;
bestFeasFval = inf;
bestAnyFval  = inf;
bestExit     = -2;
bestOutput   = [];

for iStart = 1:opts.multiStartN
    qInit   = allSeeds(:, iStart);
    runOpts = solverQuiet;
    if iStart == 1
        runOpts = solverOpts;
    end

    [qCand, fCand, exitCand, outCand] = fmincon(obj, qInit, [], [], [], [], lb, ub, nonl, runOpts);
    [cCand, ceqCand] = ik_constraints(qCand, pDes, RDes, L, opts);
    maxIneq = max([0; cCand]);
    maxEq   = max(abs(ceqCand));
    isFeas  = maxIneq <= opts.ikTol * 10 && maxEq <= opts.ikTol * 10;

    if isFeas && fCand < bestFeasFval
        bestSol      = qCand;
        bestFeasFval = fCand;
        bestExit     = exitCand;
        bestOutput   = outCand;
    elseif ~isfinite(bestFeasFval) && fCand < bestAnyFval
        bestSol     = qCand;
        bestAnyFval = fCand;
        bestExit    = exitCand;
        bestOutput  = outCand;
    end

    if isFeas
        break
    end
end

qFeasible = bestSol;

[c, ceq] = ik_constraints(qFeasible, pDes, RDes, L, opts);
[T, pAch, RAch] = fk_cache(qFeasible, L);

Rerr = RDes*RAch';
oriVec = rotm_to_rotvec(Rerr);
zAch = RAch(:,3);
zDes = RDes(:,3);

report = struct();
report.fval          = min(bestFeasFval, bestAnyFval);
report.exitflag      = bestExit;
report.output        = bestOutput;
report.maxConstraint = max([0; c]);
report.maxEqViol     = max(abs(ceq));
report.poseError     = norm(pDes - pAch);
report.oriError      = norm(oriVec);
report.dirError      = norm(cross(zAch, zDes));
report.T             = T;

end


function opts = set_defaults(opts)

if ~isfield(opts, 'boxHalfSize'); opts.boxHalfSize = [1; 1; 1.5]; end
if ~isfield(opts, 'boxMargin'); opts.boxMargin = 0.05; end
if ~isfield(opts, 'boxIgnoreFirstJoints'); opts.boxIgnoreFirstJoints = 1; end
if ~isfield(opts, 'boxCheckSamples'); opts.boxCheckSamples = 21; end
if ~isfield(opts, 'selfCollisionMinDist'); opts.selfCollisionMinDist = 0.15; end
if ~isfield(opts, 'selfCollisionSlackNeighbors'); opts.selfCollisionSlackNeighbors = 1; end
if ~isfield(opts, 'linkSamples'); opts.linkSamples = 4; end
if ~isfield(opts, 'display'); opts.display = 'off'; end
if ~isfield(opts, 'maxIterations'); opts.maxIterations = 500; end
if ~isfield(opts, 'maxFunctionEvaluations'); opts.maxFunctionEvaluations = 100000; end

if ~isfield(opts, 'wSeed'); opts.wSeed = 1.0; end
if ~isfield(opts, 'wReg'); opts.wReg = 0.01; end
if ~isfield(opts, 'multiStartN'); opts.multiStartN = 12; end
if ~isfield(opts, 'ikTol'); opts.ikTol = 1e-4; end

opts.boxHalfSize = opts.boxHalfSize(:);

end


function f = ik_objective(q, qSeed, opts)

dq = q - qSeed;
f  = opts.wSeed * dot(dq, dq) + opts.wReg * dot(q, q);

end


function [c, ceq] = ik_constraints(q, pDes, RDes, L, opts)

[~, segA, segB] = robot_geometry(q, L);

c = [];

for i = 1:size(segA, 2)
    if i <= opts.boxIgnoreFirstJoints
        continue
    end
    dSeg = segment_box_signed_distance(segA(:, i), segB(:, i), opts.boxHalfSize, opts.boxMargin, opts.boxCheckSamples);
    c(end + 1, 1) = -dSeg; %#ok<AGROW>
end

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

% Hard equality: EE position and full orientation must reach targets.
[~, p, R] = fk_cache(q, L);
Rerr = RDes*R';
oriVec = rotm_to_rotvec(Rerr);
ceq = [p - pDes; oriVec];

end


function [T, p, R] = fk_cache(q, L)

T = fkine_7dof(q, L);
p = T(1:3, 4);
R = T(1:3, 1:3);

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


function r = rotm_to_rotvec(R)

tr = trace(R);
cosTheta = (tr - 1)/2;
cosTheta = max(-1, min(1, cosTheta));
theta = acos(cosTheta);

if theta < 1e-8
    S = 0.5*(R - R');
    r = [S(3,2); S(1,3); S(2,1)];
    return
end

if abs(pi - theta) < 1e-5
    A = (R + eye(3))/2;
    v = [sqrt(max(A(1,1),0)); sqrt(max(A(2,2),0)); sqrt(max(A(3,3),0))];

    if R(3,2) - R(2,3) < 0
        v(1) = -v(1);
    end
    if R(1,3) - R(3,1) < 0
        v(2) = -v(2);
    end
    if R(2,1) - R(1,2) < 0
        v(3) = -v(3);
    end

    nv = norm(v);
    if nv < 1e-10
        v = [1;0;0];
    else
        v = v/nv;
    end

    r = theta*v;
    return
end

S = (R - R')/(2*sin(theta));
axis = [S(3,2); S(1,3); S(2,1)];
r = theta*axis;

end
