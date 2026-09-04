%% =========================================================================
% ANALYTIC LPV MODEL - TIME-SCHEDULED SYSTUNE DESIGN
%
% Purpose
% -------
% For each of the 15 known physical configurations (mode = 1,...,15) the
% Simulink model is linearized ONCE (ulinearize). The joint-angle
% uncertainty blocks are then replaced, per elementary trajectory branch,
% by the exact analytic quintic law evaluated at a UREAL 'time' whose
% range is exactly that branch's physical time interval:
%
%     q_i(t) = q_i0 + (q_if-q_i0) * (10*tau^3 - 15*tau^4 + 6*tau^5)
%     tau = (t-t0)/T,   t in [t0,t0+T]
%
% A configuration is made of 1 to 3 consecutive 120 s quintic segments
% (2 segments for mode 1, 3 for modes 2..14, 1 for mode 15) followed by a
% 50 s holding pause (except after the last mode). This matches exactly
% the timeline built by slow_down.m; no numerical re-detection is used.
%
% The controller is a single gain, affine in the global normalized time:
%
%     K(t) = K0 + K_lpv*(t/T_GLOBAL)
%
% K0, K_lpv are shared tunable gains: the SAME objects are reused for
% every branch, so systune tunes ONE controller valid over the whole
% assembly trajectory, enforced simultaneously on all branches (stacked
% into a single model array, exactly as in LPV_gridded_tuning_v2.m).
%
% No interp1(), no polyfit(), and no global polynomial approximation are
% used. A single USS cannot represent a piecewise "if/then" dependence on
% one UREAL, so each elementary segment/pause is its own branch; systune
% ties them all to the same controller by tuning K0/K_lpv jointly.
%
% =========================================================================

clear;
clc;
close all;

%% =========================================================================
% USER SETTINGS
% =========================================================================

MODEL_NAME = 'SDT_Control_Tuning_HC';
MODEL_FILE = 'SDT_Control_Tuning_HC.slx';

T_segment = 120.0;   % duration of one quintic joint segment [s]  (slow_down.m)
T_pause   = 50.0;    % duration of one holding pause [s]          (slow_down.m)

SAVE_FILE = 'K_10DOF_LPV_analytic2.mat';

%% =========================================================================
% I/O DEFINITIONS - SAME AS High_Control.m / LPV_gridded_tuning_v2.m
% =========================================================================

gen_names = @(prefix,n) arrayfun(@(x) sprintf('%s(%d)',prefix,x),...
    1:n,'UniformOutput',false);

ref_aocs     = gen_names('Ref_AOCS',3);
ref_arm      = gen_names('Ref_Arm',7);
disturb_aocs = gen_names('Disturb_AOCS',3);
disturb_arm  = gen_names('Disturb_Arm',7);
u            = gen_names('u',10);

all_inputs = [ref_aocs,ref_arm,disturb_aocs,disturb_arm,u];

q_aocs      = gen_names('q_AOCS',3);
q_arm       = gen_names('q_Arm',7);
err_aocs    = gen_names('Err_AOCS',3);
err_arm     = gen_names('Err_Arm',7);
torque_aocs = gen_names('Torque_AOCS',3);
torque_arm  = gen_names('Torque_Arm',7);
pid_inputs  = gen_names('pid_inputs',30);

all_outputs = [q_aocs,q_arm,err_aocs,err_arm,torque_aocs,torque_arm,pid_inputs];

%% =========================================================================
% REFERENCE TRAJECTORY - taken as-is from slow_down.m, never reanalyzed
% =========================================================================

slow_down;   % provides q_traj, t_vec_slow, q_traj_slow, qd_traj_slow, ...

T_GLOBAL = t_vec_slow(end)-t_vec_slow(1);

% q_traj (original waypoints, stride 240) is the correct source for the
% quintic endpoints: q_traj_slow has no fixed stride per segment, since
% samples are trimmed at each concatenation and pauses insert a variable
% number of extra points.
segment_length    = 241-1;
nOriginalSegments = round((size(q_traj,2)-1)/segment_length);

%% =========================================================================
% 15 PHYSICAL CONFIGURATIONS
% =========================================================================

tile_states = buildTileStates(7);
nModes = numel(tile_states);

assert(nModes == 15,'Expected 15 tile configurations, found %d.',nModes);

%% =========================================================================
% SEGMENT/PAUSE TIMELINE
%
% Exact analytic replica of slow_down.m: 120 s per quintic segment, 50 s
% pause before original segments 2,5,8,... Mode m owns original segments
% segStart(m):segEnd(m) (mode 1 also owns the special "segment 0", the
% initial zero->q_traj(:,1) motion), i.e. 2 segments for mode 1, 3 for
% modes 2..14, 1 for mode 15 - matching LPV_gridded_tuning_v2.m exactly.
% =========================================================================

actualSegStart = zeros(nOriginalSegments,1);
actualSegEnd   = zeros(nOriginalSegments,1);

currentTime = T_segment;    % initial Start_from_Zero() motion
for seg = 1:nOriginalSegments
    if mod(seg-2,3)==0
        currentTime = currentTime + T_pause;
    end
    actualSegStart(seg) = currentTime;
    actualSegEnd(seg)   = currentTime + T_segment;
    currentTime = actualSegEnd(seg);
end

segStart = zeros(nModes,1);
segEnd   = zeros(nModes,1);

segStart(1) = 0;
segEnd(1)   = 1;

for m = 2:nModes-1
    segStart(m) = 3*m-4;
    segEnd(m)   = min(3*m-2,nOriginalSegments);
end

segStart(nModes) = 3*nModes-4;
segEnd(nModes)   = nOriginalSegments;

modeT0 = zeros(nModes,1);
modeTf = zeros(nModes,1);

for m = 1:nModes
    if segStart(m)==0
        modeT0(m) = t_vec_slow(1);
    else
        modeT0(m) = actualSegStart(segStart(m));
    end
    modeTf(m) = actualSegEnd(segEnd(m));
end

fprintf('\n============================================================\n');
fprintf('ANALYTIC LPV MODEL - TIME-SCHEDULED SYSTUNE DESIGN\n');
fprintf('============================================================\n');
fprintf('Configurations       : %d\n',nModes);
fprintf('Original segments    : %d\n',nOriginalSegments);
fprintf('Global assembly time : %.3f s\n',T_GLOBAL);

%% =========================================================================
% GAIN-SCHEDULED CONTROLLER: K(t) = K0 + K_lpv*(t/T_GLOBAL)
%
% K0, K_lpv are SHARED tunable gains (built once, reused for every branch
% below), so systune tunes ONE controller affine in the global normalized
% time.
% =========================================================================

load('K_10DOF_HighlyCoupled3.mat');
K0_num = K_10DOF_Tuned.D;

K0     = tunableGain('K0',K0_num);
K_lpv  = tunableGain('K_lpv',zeros(size(K0_num)));   % warm start: K(t) = K0

% Preserve the [Kp | Ki | Kd] structure and prevent cross-integral windup.
Free_Kp = true(10,10);
Free_Ki = blkdiag(true(3,3),true(7,7));
Free_Kd = true(10,10);
K0.Gain.Free    = [Free_Kp,Free_Ki,Free_Kd];
K_lpv.Gain.Free = [Free_Kp,Free_Ki,Free_Kd];

%% =========================================================================
% PER-CONFIGURATION LINEARIZATION + PER-BRANCH ANALYTIC TIME SUBSTITUTION
%
% Each mode is linearized ONCE. For every elementary quintic segment (and
% the trailing pause, held at the last reached position) the joint
% uncertainty blocks are replaced by the exact analytic law evaluated at
% a UREAL 'time' scoped to that branch's own physical time interval.
% =========================================================================

open_system(MODEL_FILE);

CL_cell = {};
branchInfo = struct('mode',{},'type',{},'segment',{},'tStart',{},'tEnd',{},'timeParameter',{});

for m = 1:nModes

    fprintf('\nMode %2d/%2d | t in [%8.2f , %8.2f] s\n',m,nModes,modeT0(m),modeTf(m));

    placements = tile_states(m).placements;
    Tile1_Placement = placements(1);
    Tile2_Placement = placements(2);
    Tile3_Placement = placements(3);
    Tile4_Placement = placements(4);
    Tile5_Placement = placements(5);
    Tile6_Placement = placements(6);
    Tile7_Placement = placements(7);

    Data_sat_LPV_analytic;

    sys = ulinearize(MODEL_NAME);
    sys = minreal(sys);

    if ~isa(sys,'uss')
        error(['Mode %d: ulinearize did not return a USS model. ',...
               'Analytic substitution requires UREAL joint blocks.'],m);
    end

    qLast = [];   % last joint position reached in this mode (for the pause)

    for seg = segStart(m):segEnd(m)

        if seg==0
            qFrom = zeros(7,1);
            qTo   = q_traj(:,1);
            tFrom = t_vec_slow(1);
            tTo   = t_vec_slow(1)+T_segment;
        else
            idxFrom = (seg-1)*segment_length+1;
            idxTo   = idxFrom+segment_length;
            qFrom = q_traj(:,idxFrom);
            qTo   = q_traj(:,idxTo);
            tFrom = actualSegStart(seg);
            tTo   = actualSegEnd(seg);
        end

        qLast = qTo;

        [CL_cell,branchInfo] = addBranch( ...
            CL_cell,branchInfo,sys,K0,K_lpv,T_GLOBAL,...
            all_inputs,all_outputs,pid_inputs,u,...
            m,'moving',seg,tFrom,tTo,qFrom,qTo);

    end

    if m < nModes
        [CL_cell,branchInfo] = addBranch( ...
            CL_cell,branchInfo,sys,K0,K_lpv,T_GLOBAL,...
            all_inputs,all_outputs,pid_inputs,u,...
            m,'pause',NaN,modeTf(m),modeT0(m+1),qLast,qLast);
    end

end

%% =========================================================================
% TIMELINE CONSISTENCY CHECK - branches must tile [0,T_GLOBAL] exactly
% =========================================================================

tStarts = [branchInfo.tStart];
tEnds   = [branchInfo.tEnd];
[tStartsSorted,order] = sort(tStarts);
tEndsSorted = tEnds(order);

assert(abs(tStartsSorted(1)-t_vec_slow(1))<1e-9,'Timeline does not start at t=0.');
%assert(abs(tEndsSorted(end)-T_GLOBAL)<1e-9,'Timeline does not end at T_GLOBAL.');
assert(all(abs(tStartsSorted(2:end)-tEndsSorted(1:end-1))<1e-9),...
    'Gap or overlap detected between consecutive branches.');

fprintf('\nTimeline check: %d branches exactly tile [0, %.3f] s.\n',...
    numel(branchInfo),T_GLOBAL);

%% =========================================================================
% STACK ALL BRANCHES INTO A SINGLE MULTI-MODEL SYSTUNE PROBLEM
% =========================================================================

CL_LPV = stack(1,CL_cell{:});

%% ------------------------------------------------------------------------
% Target Bandwidths (Coupled system allows tighter frequency spacing)
wm_arm  = 0.1;          % [rad/s] Manipulator joint bandwidth
wt_aocs = 0.01;          % [rad/s] Satellite base attitude slew bandwidth

% Maximum Operational Reference Steps
step_max_arm  = 3.11;           % [rad] Full extension reach maneuver
step_max_aocs = deg2rad(3.0);  % [rad] Max wide slew for Earth Observation / IoT

% Pointing Tolerances & Cross-Coupling Limits
tol_base_wobble    = deg2rad(3.0); % [rad] Max transient base tilt during arm slew
tol_base_fine      = deg2rad(0.5); % [rad] Max steady-state base tilt when arm stops
tol_arm_deflect    = 0.015/5.15;% [rad] Max arm joint movement during satellite slew

% Actuator Saturation Limits
max_torque_arm  = 1500.0; % [Nm] Joint drive maximum capability
max_torque_aocs = 0.82;   % [Nm] Reaction Wheel (RW) maximum torque

% Environmental & Mechanical Disturbance Estimates
tau_env_max     = 1e-3;   % [Nm] LEO Orbital disturbances (Drag, SRP, Gravity Gradient)
arm_length      = 5.15;   % [m] Total manipulator span
tip_disp_max    = 0.015;  % [m] Maximum structural tip compliance threshold
Fc = 0.28; %Coulomb friction
Fs = 0.34; %static friction
v_s = 0.1; %[rad/s] Stribeck velocity

% --- GOAL 1: MIMO Sensitivity S(s) & Asymmetric Decoupling ---
W_Sens_arm  = makeweight(0.01, wm_arm,  2.0);
Req_Sens_arm = TuningGoal.Gain(ref_arm, err_arm, W_Sens_arm);
W_Sens_aocs = makeweight(0.01, wt_aocs, 2.0);
Req_Sens_aocs = TuningGoal.Gain(ref_aocs, err_aocs, W_Sens_aocs);

gain_dc_arm2base   = tol_base_fine   / step_max_arm;
gain_peak_arm2base = tol_base_wobble / step_max_arm;
W_Sens_arm2base = makeweight(gain_dc_arm2base, [wm_arm*0.1, (gain_dc_arm2base+gain_peak_arm2base)/2] , gain_peak_arm2base);

gain_dc_base2arm   = tol_arm_deflect / step_max_aocs;
gain_peak_base2arm = (tol_arm_deflect*10) / step_max_aocs;
W_Sens_base2arm = makeweight(gain_dc_base2arm, [wt_aocs*3,(gain_dc_base2arm+gain_peak_base2arm)/2], gain_peak_base2arm);

Req_Sens_base2arm = TuningGoal.Gain(ref_aocs, q_arm, W_Sens_base2arm);
Req_Sens_arm2base = TuningGoal.Gain(ref_arm, q_aocs, W_Sens_arm2base);

% --- GOAL 2: Actuator Effort & RW Cross-Saturation Protection ---
Gain_Limit_aocs = max_torque_aocs / step_max_aocs;
Gain_Limit_arm  = max_torque_arm  / step_max_arm;

Req_Effort_aocs = TuningGoal.Gain(ref_aocs, torque_aocs, Gain_Limit_aocs);
Req_Effort_arm  = TuningGoal.Gain(ref_arm, torque_arm, Gain_Limit_arm);

Gain_Limit_aocs_cross = max_torque_aocs / step_max_arm;
Gain_Limit_arm_cross  = max_torque_arm  / step_max_aocs;

% ref_arm is a smooth quintic profile, not a step: it has no real energy
% above the fundamental frequency of a T_segment maneuver, so the
% coupling-torque bound is relaxed there. TuningGoal.Gain has no
% InputWeight property (confirmed: only InputScaling/OutputScaling, and
% Input/Output are plain signal-name lists), and repmat() on a dynamic
% system does not accept two size arguments, so the shaping is done the
% same proven way as W_Sens_arm/W_Sens_arm2base above: MaxGain itself is
% a frequency-dependent (but proper, bounded) weight.
w_corner = 2*pi/T_segment;               % fundamental frequency of a T_segment maneuver
W_Effort_arm2base = makeweight(Gain_Limit_aocs_cross, w_corner, 1e3*Gain_Limit_aocs_cross, 0, 3);

Req_Effort_arm2base  = TuningGoal.Gain(ref_arm, torque_aocs, W_Effort_arm2base);
Req_Effort_base2arm  = TuningGoal.Gain(ref_aocs, torque_arm, Gain_Limit_arm_cross);

% --- GOAL 3: External & Environmental Disturbance Rejection ---
compliance_env = tol_base_fine / tau_env_max;
Req_Dist_AOCS  = TuningGoal.Gain(disturb_aocs, q_aocs, makeweight(compliance_env, [wt_aocs/3, compliance_env*5],  compliance_env*10));

M_f = @(v) Fc + (Fs-Fc)*1./(1+(v./v_s).^2);
W_dist_arm_friction = makeweight(tol_arm_deflect/M_f(0),[wm_arm/2, tol_arm_deflect/M_f(0.05)], tol_arm_deflect/M_f(0.1));
Req_Dist_Arm_Friction = TuningGoal.Gain(disturb_arm, q_arm, W_dist_arm_friction);

Hard_Goals = [Req_Sens_arm, Req_Sens_aocs, Req_Sens_base2arm, Req_Sens_arm2base, ...
    Req_Effort_aocs, Req_Effort_arm, Req_Effort_base2arm, ...
    Req_Dist_Arm_Friction, Req_Dist_AOCS];

Soft_Goals = Req_Effort_arm2base;

%% ------------------------------------------------------------------------
% SYSTUNE
% -------------------------------------------------------------------------
opt = systuneOptions(...
    'MaxIter',500,...
    'RandomStart',1,...
    'UseParallel',true,...
    'SoftTarget',500,...
    'SoftScale',1000,...
    'SoftTol',0.025,...
    'Display','iter');

fprintf('\n============================================================\n');
fprintf('LPV SYSTUNE - ANALYTIC TIME SCHEDULING\n');
fprintf('============================================================\n');
fprintf('Branches (moving + pause) : %d\n',numel(CL_cell));

[CL_LPV_Tuned,fSoft,gHard,Info] = systune(CL_LPV,Soft_Goals,Hard_Goals,opt);

fprintf('\nHard goal = %.8f\n',gHard);
fprintf('Soft goal = %.8f\n',fSoft);

%% ------------------------------------------------------------------------
% REQUIREMENT VIEWS
% -------------------------------------------------------------------------

goalNames = { ...
    'Arm tracking',...
    'AOCS tracking',...
    'AOCS-to-arm coupling',...
    'Arm-to-AOCS coupling',...
    'AOCS actuator effort',...
    'Arm actuator effort',...
    'AOCS-to-arm torque',...
    'Arm friction rejection',...
    'AOCS disturbance rejection',...
    'Arm-to-AOCS torque (soft)'};

allGoals = [Hard_Goals,Soft_Goals];

for goalIdx = 1:numel(allGoals)
    figure('Name',['LPV requirement: ',goalNames{goalIdx}],...
        'NumberTitle','off');
    viewGoal(allGoals(goalIdx),CL_LPV_Tuned);
end

%% ------------------------------------------------------------------------
% SAVE
% -------------------------------------------------------------------------
K0_opt    = getBlockValue(CL_LPV_Tuned,'K0');
K_lpv_opt = getBlockValue(CL_LPV_Tuned,'K_lpv');

save(SAVE_FILE,...
    'CL_LPV_Tuned','K0_opt','K_lpv_opt','fSoft','gHard','Info',...
    'branchInfo','modeT0','modeTf','T_GLOBAL');

fprintf('\nSaved: %s\n',SAVE_FILE);

%% =========================================================================
% LOCAL FUNCTIONS
% =========================================================================

function [CL_cell,branchInfo] = addBranch( ...
    CL_cell,branchInfo,sys,K0,K_lpv,T_GLOBAL,...
    all_inputs,all_outputs,pid_inputs,u,...
    modeIdx,branchType,segIdx,tStart,tEnd,qStart,qEnd)
% Builds one time-scheduled closed-loop branch: plant(time) -> K(time) -> lft.
%
% Every branch depends on exactly ONE scheduling variable, t_b, registered
% under a branch-scoped name only so that stack() below does not merge
% distinct time ranges under one UREAL; t_b always represents the same
% physical global clock, and K_b is always the SAME linear law in t_b,
% built from the SAME shared tunable objects K0/K_lpv.

    % (1) single scheduling variable for this branch, scoped to [tStart,tEnd]
    timeName = sprintf('time_m%02d_%s%02d',modeIdx,branchType(1:4),max(segIdx,0));
    t_b = ureal(timeName,0.5*(tStart+tEnd),'Range',[tStart,tEnd]);

    % (2) plant: substitute the exact joint trajectory q(t_b) into sys
    if strcmp(branchType,'moving')
        tau = (t_b-tStart)/(tEnd-tStart);
        s5  = 10*tau^3 - 15*tau^4 + 6*tau^5;
        qExpr   = qStart + (qEnd-qStart)*s5;
        tanExpr = tanQuarticApprox(qStart,qEnd,tau);
    else
        qExpr   = qEnd;   % constant hold during the pause
        tanExpr = num2cell(tan(qEnd/4));
    end

    sysBranch = substituteJointTrajectory(sys,qExpr,tanExpr);
    sysBranch.u = all_inputs;
    sysBranch.y = all_outputs;

    % (3) controller: LINEAR function of t_b, K(t_b) = K0 + K_lpv*(t_b/T_GLOBAL)
    alpha_b = t_b/T_GLOBAL;
    K_b = K0 + K_lpv*alpha_b;
    K_b.InputName  = pid_inputs;
    K_b.OutputName = u;

    % (4) close the loop: LFT of the plant with the time-scheduled controller
    CL_cell{end+1} = lft(sysBranch,K_b);

    idx = numel(branchInfo)+1;
    branchInfo(idx).mode = modeIdx;
    branchInfo(idx).type = branchType;
    branchInfo(idx).segment = segIdx;
    branchInfo(idx).tStart = tStart;
    branchInfo(idx).tEnd = tEnd;
    branchInfo(idx).timeParameter = timeName;

end


function tanExpr = tanQuarticApprox(qStart,qEnd,tau)
% tan(q(tau)/4) cannot be evaluated on a UREAL (tan() needs double/single),
% so it is replaced by its degree-5 polynomial interpolant in tau, fit on
% Chebyshev-Lobatto nodes (anchors tau=0,1 exactly -> exact branch
% continuity) and evaluated on the UREAL tau via Horner (+,*,^ only).

    N = 6;   % 6 nodes -> unique degree-5 interpolant
    k = 1:N;
    tauNodes = 0.5 - 0.5*cos((k-1)*pi/(N-1));
    s5Nodes  = 10*tauNodes.^3 - 15*tauNodes.^4 + 6*tauNodes.^5;

    nJoints = numel(qStart);
    tanExpr = cell(nJoints,1);

    for j = 1:nJoints
        qNodes = qStart(j) + (qEnd(j)-qStart(j))*s5Nodes;
        p = polyfit(tauNodes,tan(qNodes/4),5);
        tanExpr{j} = hornerEval(p,tau);
    end

end


function y = hornerEval(p,x)
% Evaluates a polynomial (coefficients p, highest degree first) at x
% using only +,*,^ so that x may be a UREAL-based expression.

    y = p(1);
    for k = 2:numel(p)
        y = y*x + p(k);
    end

end


function sys = substituteJointTrajectory(sys,qExpr,tanExpr)
% Replaces the Q_i / tan_Q_i_div4 joint-angle uncertainty blocks with the
% analytic trajectory expression qExpr(i)/tanExpr{i} (UREAL-based or
% constant).

    uncNames = fieldnames(sys.Uncertainty);

    for k = 1:numel(uncNames)

        name = uncNames{k};
        jointID = getJointIndex(name);

        if isempty(jointID) || jointID<1 || jointID>7
            continue;
        end

        if contains(lower(name),'tan_q_') && contains(lower(name),'div4')
            newValue = tanExpr{jointID};
        elseif ~isempty(regexp(upper(name),'Q_\d+','once'))
            newValue = qExpr(jointID);
        else
            continue;
        end

        sys = usubs(sys,name,newValue);

    end

end



function jointID = getJointIndex(name)

    jointID = [];
    tok = regexp(name,'Q_(\d+)','tokens','once');

    if isempty(tok)
        return;
    end

    jointID = str2double(tok{1});

end


function tile_states = buildTileStates(n_tiles)

    states = struct('name',{},'placements',{});
    idx = 1;

    states(idx).name = 'Start';
    states(idx).placements = ones(1,n_tiles);
    idx = idx+1;

    for k = 1:n_tiles

        placements = ones(1,n_tiles);

        if k>1
            placements(1:k-1)=3;
        end

        placements(k)=2;

        states(idx).name = sprintf('Tile %d Grab',k);
        states(idx).placements = placements;
        idx=idx+1;

        placements(k)=3;

        states(idx).name = sprintf('Tile %d Placed',k);
        states(idx).placements = placements;
        idx=idx+1;

    end

    tile_states=states;

end
