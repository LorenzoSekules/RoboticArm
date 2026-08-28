%% LPV_GRIDDED_GLOBAL_ALPHA.M
% 7-DOF Arm + 3-DOF AOCS
%
% GRID-BASED LPV WITH ONE CONTINUOUS GLOBAL SCHEDULING VARIABLE
% ================================================================
%
% Idea:
%   - There are 15 physical assembly configurations (modes).
%   - m is NOT a scheduling variable and is NOT interpolated.
%   - Within each configuration, take N frozen linearizations.
%   - All frozen models are assigned a single GLOBAL continuous parameter
%       alpha in [0,1], obtained from the absolute mission time:
%       alpha = (t-tStart)/(tEnd-tStart).
%   - The gain schedule is therefore K(alpha), continuous through the whole
%     assembly process.
%
% Plant data structure:
%       G(alpha_k), k = 1,...,Ntotal
% where each alpha_k belongs to exactly one physical configuration m.
%
% Controller:
%       K(alpha) = K0 + K1*F1(alpha) + ...
% implemented with tunableSurface.
%
% Initially use ORDER = 1 (affine). Then compare ORDER = 2,3,5.
%
% IMPORTANT:
%   The 15 configurations are still visible in the SamplingGrid as the
% bookkeeping field "mode", but mode is NOT a scheduling variable of the
% tunable surface. Therefore the controller is one continuous function of
% alpha only.
%
% Requirements from existing project:
%   SDT_Control_Tuning_HC.slx
%   Data_sat_Nominal.m
%   best_trajectory_def.mat
%   K_10DOF_HighlyCoupled_LU.mat
%
% The operating-point variables q_i, qd_i, qdd_i are assumed to be used by
% the Simulink model as in High_Control.m.

clear; clc; close all;

%% ========================================================================
% USER SETTINGS
% =========================================================================

N_PER_MODE = 10;             % requested ~10 frozen LTI points/configuration
SHAPE_ORDER = 1;             % 1: affine, 2: quadratic, 3: cubic, 5: quintic

RUN_TUNING = true;
SAVE_RESULTS = true;

modelName = 'SDT_Control_Tuning_HC';
modelFile = [modelName '.slx'];
initialControllerFile = 'K_10DOF_HighlyCoupled_LU.mat';

nArm = 7;
nAocs = 3;
nDOF = 10;
nPID = 30;

%% ========================================================================
% I/O NAMES -- identical to High_Control.m
% =========================================================================

gen_names = @(prefix,n) arrayfun(@(x) sprintf('%s(%d)',prefix,x),1:n,'UniformOutput',false);

ref_aocs     = gen_names('Ref_AOCS',3);
ref_arm      = gen_names('Ref_Arm',7);
disturb_aocs = gen_names('Disturb_AOCS',3);
disturb_arm  = gen_names('Disturb_Arm',7);
u            = gen_names('u',10);
all_inputs   = [ref_aocs,ref_arm,disturb_aocs,disturb_arm,u];

q_aocs      = gen_names('q_AOCS',3);
q_arm       = gen_names('q_Arm',7);
err_aocs    = gen_names('Err_AOCS',3);
err_arm     = gen_names('Err_Arm',7);
torque_aocs = gen_names('Torque_AOCS',3);
torque_arm  = gen_names('Torque_Arm',7);
pid_inputs  = gen_names('pid_inputs',30);
all_outputs = [q_aocs,q_arm,err_aocs,err_arm,torque_aocs,torque_arm,pid_inputs];

%% ========================================================================
% LOAD / PREPARE TRAJECTORY
% =========================================================================

load('best_trajectory_def.mat');

traj_options = struct();
traj_options.totalTime = 12.0;
traj_options.numSamples = 241;

% Same initial zero-to-first-waypoint transition as High_Control.m
q_start_zero = zeros(7,1);
q_first_wp = q_traj(:,1);

[t_init,q_init,qd_init,qdd_init] = trajectoryGeneration_local( ...
    q_start_zero,q_first_wp,traj_options);

q_traj   = [q_init(:,1:end-1),q_traj];
qd_traj  = [qd_init(:,1:end-1),qd_traj];
qdd_traj = [qdd_init(:,1:end-1),qdd_traj];
t_vec    = [t_init(1:end-1),t_vec+t_init(end)];

numSamples = 241;
segment_length = numSamples-1;
nTotal = size(q_traj,2);

if size(qd_traj,2) ~= nTotal || size(qdd_traj,2) ~= nTotal || numel(t_vec) ~= nTotal
    error('q/qd/qdd/t trajectory arrays are inconsistent.');
end

nSegments = (nTotal-1)/segment_length;
if abs(nSegments-round(nSegments)) > 1e-12
    error('Trajectory length is inconsistent with numSamples=%d.',numSamples);
end
nSegments = round(nSegments);

% Global continuous scheduling variable.
tStart = t_vec(1);
tEnd   = t_vec(end);
alphaAll = (t_vec-tStart)/(tEnd-tStart);

if alphaAll(1) ~= 0 || abs(alphaAll(end)-1) > 1e-12
    error('Could not construct global alpha in [0,1].');
end

%% ========================================================================
% 15 DISCRETE ASSEMBLY CONFIGURATIONS
% =========================================================================

tile_states = buildTileStates_local(7);
nModes = numel(tile_states);

if nModes ~= 15
    error('Expected 15 assembly configurations, found %d.',nModes);
end

%% ========================================================================
% OPEN MODEL
% =========================================================================

if ~isfile(modelFile)
    error('Missing Simulink model: %s',modelFile);
end
open(modelFile);

%% ========================================================================
% BUILD GRID: N_PER_MODE FROZEN LINEARIZATIONS PER CONFIGURATION
% =========================================================================

fprintf('\n==============================================================\n');
fprintf('GRID-BASED LPV WITH GLOBAL CONTINUOUS ALPHA\n');
fprintf('15 configurations x %d points/configuration\n',N_PER_MODE);
fprintf('Expected frozen models: %d\n',nModes*N_PER_MODE);
fprintf('alpha = (t-tStart)/(tEnd-tStart) in [0,1]\n');
fprintf('Controller shape order = %d\n',SHAPE_ORDER);
fprintf('==============================================================\n');

% We store one model per point in one vector. This avoids treating mode as
% a scheduling coordinate or creating a rectangular (mode,alpha) grid.
G_cell = cell(nModes*N_PER_MODE,1);
alphaGrid = zeros(nModes*N_PER_MODE,1);
modeGrid  = zeros(nModes*N_PER_MODE,1);
timeGrid  = zeros(nModes*N_PER_MODE,1);

counter = 0;

for m = 1:nModes

    [segStart,segEnd] = localWindowForMode_local(m);

    idxStart = segStart*segment_length + 1;
    idxEnd   = segEnd*segment_length + 1;

    if idxStart < 1 || idxEnd > nTotal
        error('Mode %d maps outside trajectory: [%d,%d].',m,idxStart,idxEnd);
    end

    % Absolute time interval occupied by this configuration.
    tMode = [t_vec(idxStart),t_vec(idxEnd)];

    % Avoid duplicating a transition time in two consecutive configurations.
    % Every mode still gets exactly N_PER_MODE design points.
    if m == 1
        tSamples = linspace(tMode(1),tMode(2),N_PER_MODE);
    else
        tSamples = linspace(tMode(1),tMode(2),N_PER_MODE);
        tSamples(1) = tMode(1) + (tMode(2)-tMode(1))*eps;
    end

    fprintf('\nMode %2d/%2d: %s\n',m,nModes,tile_states(m).name);
    fprintf('  trajectory indices : %d -> %d\n',idxStart,idxEnd);
    fprintf('  time interval      : %.6f -> %.6f s\n',tMode(1),tMode(2));
    fprintf('  alpha interval     : %.6f -> %.6f\n', ...
        (tMode(1)-tStart)/(tEnd-tStart), ...
        (tMode(2)-tStart)/(tEnd-tStart));

    % Configure physical assembly state.
    placements = tile_states(m).placements;
    Tile1_Placement = placements(1); %#ok<NASGU>
    Tile2_Placement = placements(2); %#ok<NASGU>
    Tile3_Placement = placements(3); %#ok<NASGU>
    Tile4_Placement = placements(4); %#ok<NASGU>
    Tile5_Placement = placements(5); %#ok<NASGU>
    Tile6_Placement = placements(6); %#ok<NASGU>
    Tile7_Placement = placements(7); %#ok<NASGU>

    for k = 1:N_PER_MODE

        t = tSamples(k);

        % Continuous global scheduling parameter used by the controller.
        alpha = (t-tStart)/(tEnd-tStart);

        % Interpolate the actual trajectory at this absolute time.
        q_i = interp1(t_vec,q_traj.',t,'pchip').';       %#ok<NASGU>
        qd_i = interp1(t_vec,qd_traj.',t,'pchip').';    %#ok<NASGU>
        qdd_i = interp1(t_vec,qdd_traj.',t,'pchip').';  %#ok<NASGU>

        % Existing project initialization.
        Data_sat_Nominal;

        % Freeze and linearize the plant at this operating point.
        sys = ulinearize(modelName);
        sys = minreal(sys);

        % IMPORTANT:
        % Keep the uncertainty handling of High_Control.m. Each frozen model
        % gets independent copies of the local Q_ uncertainty blocks.
        if isa(sys,'uss')
            unc_names = fieldnames(sys.Uncertainty);

            for v = 1:numel(unc_names)
                oldName = unc_names{v};

                if contains(oldName,'Q_')
                    oldU = sys.Uncertainty.(oldName);

                    newName = sprintf('%s_mode%d_p%02d',oldName,m,k);

                    newU = ureal( ...
                        newName,...
                        oldU.NominalValue,...
                        'Range',oldU.Range);

                    sys = usubs(sys,oldName,newU);
                end
            end
        end

        sys.u = all_inputs;
        sys.y = all_outputs;

        counter = counter+1;
        G_cell{counter} = sys;
        alphaGrid(counter) = alpha;
        modeGrid(counter) = m;
        timeGrid(counter) = t;

        fprintf('    point %2d/%2d: t=%9.4f s, alpha=%8.6f\n', ...
            k,N_PER_MODE,t,alpha);
    end
end

% Sort explicitly by global alpha, just to guarantee monotonic ordering.
[alphaGrid,sortIdx] = sort(alphaGrid,'ascend');
modeGrid = modeGrid(sortIdx);
timeGrid = timeGrid(sortIdx);
G_cell = G_cell(sortIdx);

% Verify strictly increasing alpha. There must be no duplicated scheduling
% point because the controller surface is single-valued in alpha.
dAlpha = diff(alphaGrid);
if any(dAlpha <= 0)
    error('alpha grid is not strictly increasing. Duplicate transition point detected.');
end

G_grid = stack(1,G_cell{:});
G_grid.SamplingGrid = struct( ...
    'alpha',alphaGrid,...
    'mode',modeGrid,...
    'time',timeGrid);

fprintf('\nGrid complete: %d frozen plant models.\n',counter);
fprintf('alpha range: [%.6f, %.6f]\n',alphaGrid(1),alphaGrid(end));

%% ========================================================================
% INITIAL CONTROLLER
% =========================================================================

if ~isfile(initialControllerFile)
    error('Missing initial controller file: %s',initialControllerFile);
end

load(initialControllerFile,'K_10DOF_Tuned');

if ~exist('K_10DOF_Tuned','var') || ~isequal(size(K_10DOF_Tuned),[10 30])
    error('K_10DOF_Tuned must be a numeric 10x30 matrix.');
end

Kp0 = K_10DOF_Tuned(:,1:10);
Ki0 = K_10DOF_Tuned(:,11:20);
Kd0 = K_10DOF_Tuned(:,21:30);

%% ========================================================================
% GAIN SURFACE K(alpha)
% =========================================================================

% IMPORTANT:
% The surface has ONLY ONE scheduling variable: alpha.
% "mode" is merely metadata attached to the plant array.
%
% tunableSurface internally normalizes alpha using the design points. Thus
% the basis is evaluated in the normalized scheduling coordinate n(alpha).
% This does NOT make alpha discontinuous; the resulting K(alpha) remains
% continuous as alpha varies.

domain = struct('alpha',alphaGrid);
shapeFun = makeShapeFunction_local(SHAPE_ORDER);

Kp = tunableSurface('Kp_LPV',Kp0,domain,shapeFun);
Ki = tunableSurface('Ki_LPV',Ki0,domain,shapeFun);
Kd = tunableSurface('Kd_LPV',Kd0,domain,shapeFun);

% Use the physical global alpha directly in the basis functions.
% By default tunableSurface normalizes scheduling variables to [-1,1].
% Here we explicitly choose n(alpha)=alpha, so the tuned law is literally
%     K(alpha) = K0 + K1*alpha + K2*alpha^2 + ...
% which makes the same alpha(t) directly usable in the Simulink schedule.
Kp.Normalization.InputOffset = 0;
Kp.Normalization.InputScaling = 1;
Ki.Normalization.InputOffset = 0;
Ki.Normalization.InputScaling = 1;
Kd.Normalization.InputOffset = 0;
Kd.Normalization.InputScaling = 1;

% Kp, Ki, Kd each use ONLY their own 10 measurement channels.
% The final controller is the same 10x30 [Kp Ki Kd] structure as in
% High_Control.m.
Kp.InputName = pid_inputs(1:10);
Kp.OutputName = u;
Ki.InputName = pid_inputs(11:20);
Ki.OutputName = u;
Kd.InputName = pid_inputs(21:30);
Kd.OutputName = u;

% Preserve the integral-gain block-diagonal structure used in High_Control.m.
% tunableSurface stores all basis coefficients in one array-valued realp.
% We zero/freeze off-block entries for every coefficient.
enforceSurfaceMask_local(Ki,...
    [true(3,3),false(3,7);...
     false(7,3),true(7,7)]);

% IMPORTANT: horizontal concatenation, not addition.
% Each scheduled block acts on a different 10-channel portion of pid_inputs.
K_LPV = [Kp,Ki,Kd];
K_LPV.InputName = pid_inputs;
K_LPV.OutputName = u;

%% ========================================================================
% CLOSE LOOP
% =========================================================================

G_full = G_grid(all_outputs,all_inputs);
CL_grid = lft(G_full,K_LPV);

%% ========================================================================
% SAME MISSION REQUIREMENTS AS High_Control.m
% =========================================================================

wm_arm  = 0.1;
wt_aocs = 0.01;

step_max_arm  = 3.11;
step_max_aocs = deg2rad(3.0);

tol_base_wobble = deg2rad(3.0);
tol_base_fine   = deg2rad(0.5);
tol_arm_deflect = 0.015/5.15;

max_torque_arm  = 1500.0;
max_torque_aocs = 0.82;

tau_env_max = 1e-3;
Fc = 0.28;
Fs = 0.34;
v_s = 0.1;

W_Sens_arm = makeweight(0.01,wm_arm,2.0);
Req_Sens_arm = TuningGoal.Gain(ref_arm,err_arm,W_Sens_arm);

W_Sens_aocs = makeweight(0.01,wt_aocs,2.0);
Req_Sens_aocs = TuningGoal.Gain(ref_aocs,err_aocs,W_Sens_aocs);

gain_dc_arm2base   = tol_base_fine/step_max_arm;
gain_peak_arm2base = tol_base_wobble/step_max_arm;
W_Sens_arm2base = makeweight( ...
    gain_dc_arm2base,...
    [wm_arm*0.1,(gain_dc_arm2base+gain_peak_arm2base)/2],...
    gain_peak_arm2base);
Req_Sens_arm2base = TuningGoal.Gain(ref_arm,q_aocs,W_Sens_arm2base);

gain_dc_base2arm   = tol_arm_deflect/step_max_aocs;
gain_peak_base2arm = (tol_arm_deflect*10)/step_max_aocs;
W_Sens_base2arm = makeweight( ...
    gain_dc_base2arm,...
    [wt_aocs*3,(gain_dc_base2arm+gain_peak_base2arm)/2],...
    gain_peak_base2arm);
Req_Sens_base2arm = TuningGoal.Gain(ref_aocs,q_arm,W_Sens_base2arm);

Gain_Limit_aocs = max_torque_aocs/step_max_aocs;
Gain_Limit_arm  = max_torque_arm/step_max_arm;
Req_Effort_aocs = TuningGoal.Gain(ref_aocs,torque_aocs,Gain_Limit_aocs);
Req_Effort_arm  = TuningGoal.Gain(ref_arm,torque_arm,Gain_Limit_arm);

Gain_Limit_aocs_cross = max_torque_aocs/step_max_arm;
Gain_Limit_arm_cross  = max_torque_arm/step_max_aocs;
Req_Effort_arm2base = TuningGoal.Gain(ref_arm,torque_aocs,Gain_Limit_aocs_cross);
Req_Effort_base2arm = TuningGoal.Gain(ref_aocs,torque_arm,Gain_Limit_arm_cross);

compliance_env = tol_base_fine/tau_env_max;
Req_Dist_AOCS = TuningGoal.Gain( ...
    disturb_aocs,q_aocs,...
    makeweight(compliance_env,[wt_aocs/3,compliance_env*5],compliance_env*10));

M_f = @(v) Fc+(Fs-Fc)./(1+(v./v_s).^2);
W_dist_arm_friction = makeweight( ...
    tol_arm_deflect/M_f(0),...
    [wm_arm/2,tol_arm_deflect/M_f(0.05)],...
    tol_arm_deflect/M_f(0.1));
Req_Dist_Arm_Friction = TuningGoal.Gain(...
    disturb_arm,q_arm,W_dist_arm_friction);

Hard_Goals = [ ...
    Req_Sens_arm,...
    Req_Sens_aocs,...
    Req_Sens_base2arm,...
    Req_Sens_arm2base,...
    Req_Effort_aocs,...
    Req_Effort_arm,...
    Req_Effort_base2arm,...
    Req_Dist_Arm_Friction,...
    Req_Dist_AOCS];

Soft_Goals = Req_Effort_arm2base;

%% ========================================================================
% TUNING
% =========================================================================

if RUN_TUNING

    opt = systuneOptions(...
        'MaxIter',500,...
        'RandomStart',1,...
        'UseParallel',true,...
        'SoftTarget',1200,...
        'SoftScale',1700,...
        'SoftTol',0.07,...
        'Display','iter');

    fprintf('\n==============================================================\n');
    fprintf('STARTING GRID-BASED LPV TUNING\n');
    fprintf('One continuous scheduling variable: alpha\n');
    fprintf('Number of plant models: %d\n',counter);
    fprintf('==============================================================\n');

    [CL_grid_Tuned,fSoft,gHard,Info] = systune(...
        CL_grid,Soft_Goals,Hard_Goals,opt); %#ok<ASGLU>

    fprintf('\n==============================================================\n');
    fprintf('GRID-BASED LPV RESULTS\n');
    fprintf('==============================================================\n');
    fprintf('Hard goal peak : %.6f\n',gHard);
    fprintf('Soft goal      : %.6f\n',fSoft);

    % Optional surface visualization.
    try
        fprintf('Use viewSurf() on Kp/Ki/Kd from the tuned model to inspect K(alpha).\n');
    catch
    end

    if SAVE_RESULTS
        resultFile = sprintf('K_10DOF_LPV_gridded_alpha_N%d_order%d.mat', ...
            N_PER_MODE,SHAPE_ORDER);

        Kp_opt = getBlockValue(CL_grid_Tuned,'Kp_LPV');
        Ki_opt = getBlockValue(CL_grid_Tuned,'Ki_LPV');
        Kd_opt = getBlockValue(CL_grid_Tuned,'Kd_LPV');

        save(resultFile,...
            'Kp_opt','Ki_opt','Kd_opt',...
            'alphaGrid','modeGrid','timeGrid',...
            'N_PER_MODE','SHAPE_ORDER',...
            'gHard','fSoft','Info');

        fprintf('Saved results to %s\n',resultFile);
    end
end

%% ========================================================================
% LOCAL FUNCTIONS
% =========================================================================

function [seg_start,seg_end] = localWindowForMode_local(m)
% Same 15-mode bookkeeping used in High_Control.m.
% Segment numbering starts at zero.

    if m == 1
        seg_start = 0;
        seg_end   = 2;
    elseif m == 15
        seg_start = 41;
        seg_end   = 42;
    else
        seg_start = 3*m-4;
        seg_end   = 3*m-1;
    end
end

function tile_states = buildTileStates_local(n_tiles)

    states = struct('name',{},'placements',{});
    idx = 1;

    states(idx).name = 'Start';
    states(idx).placements = ones(1,n_tiles);
    idx = idx+1;

    for k = 1:n_tiles

        placements = ones(1,n_tiles);

        if k > 1
            placements(1:k-1) = 3;
        end

        placements(k) = 2;
        states(idx).name = sprintf('Tile %d Grab',k);
        states(idx).placements = placements;
        idx = idx+1;

        placements(k) = 3;
        states(idx).name = sprintf('Tile %d Placed',k);
        states(idx).placements = placements;
        idx = idx+1;
    end

    tile_states = states;
end

function shapefcn = makeShapeFunction_local(order)

    switch order
        case 0
            shapefcn = [];
        case 1
            shapefcn = @(x) x;
        case 2
            shapefcn = @(x) [x,x.^2];
        case 3
            shapefcn = @(x) [x,x.^2,x.^3];
        case 4
            shapefcn = @(x) [x,x.^2,x.^3,x.^4];
        case 5
            shapefcn = @(x) [x,x.^2,x.^3,x.^4,x.^5];
        otherwise
            shapefcn = @(x) x.^(1:order);
    end
end

function enforceSurfaceMask_local(K,freeMask)
% Freeze selected coefficient entries of a tunableSurface.
%
% For a matrix-valued tunableSurface, K.Coefficients is an array-valued
% realp with horizontal concatenation of all coefficient matrices:
%   [K0 K1 ... KM]
%
% We use the realp.Free mask on each block-sized horizontal section.

    nBasis = size(K.Coefficients,2)/size(freeMask,2);
    if abs(nBasis-round(nBasis)) > 1e-12
        error('Unexpected tunableSurface coefficient dimensions.');
    end
    nBasis = round(nBasis);

    P = K.Coefficients;
    for b = 1:nBasis
        cols = (b-1)*size(freeMask,2)+(1:size(freeMask,2));
        P.Free(:,cols) = freeMask;
    end
    K.Coefficients = P;
end

function [t,q,qd,qdd] = trajectoryGeneration_local(q_start,q_goal,options)

    T = options.totalTime;
    n_samples = options.numSamples;

    t = linspace(0,T,n_samples);
    tau = t/T;

    s = 10*tau.^3-15*tau.^4+6*tau.^5;
    s_dot = (30*tau.^2-60*tau.^3+30*tau.^4)/T;
    s_ddot = (60*tau-180*tau.^2+120*tau.^3)/(T^2);

    delta_q = q_goal-q_start;

    q = q_start+delta_q*s;
    qd = delta_q*s_dot;
    qdd = delta_q*s_ddot;
end
