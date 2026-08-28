%% LPV_ANALYTIC_PIECEWISE_TUNING.M
% 7-DOF arm + 3-DOF AOCS
% Analytic / doctoral-style experiment with piecewise quintic trajectories.
%
% IMPORTANT MODELING POINT
% ------------------------
% The doctoral script works with a PARAMETRIC linearized plant whose
% uncertain parameters are explicit functions of the scheduling variable.
% A frozen ulinearize() result does not contain that functional dependence.
%
% Therefore this file provides two modes:
%
%   ANALYTIC_SOURCE = 'PARAMETRIC'
%       Exact doctoral-style route. Requires the user's Simulink/model setup
%       to expose the linearized plant parameters as uncertain/real parameters.
%       Implement that model-specific mapping in getParametricPlantMode().
%
%   ANALYTIC_SOURCE = 'FIT_LTI'
%       Runnable fallback with the current workflow: generate frozen LTI/USS
%       models on each quintic segment, fit A,B,C,D entry-wise with degree P,
%       and construct a continuous uncertain/real-parameter surrogate.
%       This reproduces the ANALYTIC SURROGATE idea, but does NOT preserve the
%       original structured USS uncertainty exactly. Use it for method
%       comparison/debugging, not as the final robust certificate.
%
% Scheduling variable:
%   s in [0,1] LOCAL TO EACH QUINTIC SEGMENT.
%
% Plant family:
%   15 discrete configurations x 3 local segments = 45 branches.
%
% Controller:
%   K(s) = K0 + s*KL*KR
% where KL*KR is a low-rank scheduling variation, as in the doctoral script.
%
% The same controller coefficients are used for all 15 configurations and
% all piecewise branches.

clear; clc; close all;

%% ========================================================================
% USER SETTINGS
% =========================================================================

ANALYTIC_SOURCE = 'FIT_LTI';       % 'PARAMETRIC' or 'FIT_LTI'

P = 5;                              % trajectory polynomial degree
Pplant = 5;                         % fallback plant polynomial degree
Nfit = 9;                            % frozen points used to fit each branch
rankK = 5;                          % low-rank factorization KL*KR

RUN_TUNING = true;
SAVE_RESULTS = true;

modelName = 'SDT_Control_Tuning_HC';
modelFile = [modelName '.slx'];
initialControllerFile = 'K_10DOF_HighlyCoupled_LU.mat';

nArm = 7;
nAocs = 3;
nDOF = nArm+nAocs;
nPID = 3*nDOF;

%% ========================================================================
% I/O NAMES -- identical to High_Control.m
% =========================================================================

gen_names = @(prefix,n) arrayfun(@(x) sprintf('%s(%d)',prefix,x),1:n,'UniformOutput',false);

ref_aocs = gen_names('Ref_AOCS',3);
ref_arm = gen_names('Ref_Arm',7);
disturb_aocs = gen_names('Disturb_AOCS',3);
disturb_arm = gen_names('Disturb_Arm',7);
u = gen_names('u',10);
all_inputs = [ref_aocs,ref_arm,disturb_aocs,disturb_arm,u];

q_aocs = gen_names('q_AOCS',3);
q_arm = gen_names('q_Arm',7);
err_aocs = gen_names('Err_AOCS',3);
err_arm = gen_names('Err_Arm',7);
torque_aocs = gen_names('Torque_AOCS',3);
torque_arm = gen_names('Torque_Arm',7);
pid_inputs = gen_names('pid_inputs',30);
all_outputs = [q_aocs,q_arm,err_aocs,err_arm,torque_aocs,torque_arm,pid_inputs];

%% ========================================================================
% TRAJECTORY
% =========================================================================

load('best_trajectory_def.mat');

traj_options.totalTime = 12.0;
traj_options.numSamples = 241;
q_start_zero = zeros(7,1);
q_first_wp = q_traj(:,1);
[t_init,q_init,qd_init,qdd_init] = trajectoryGeneration_local(...
    q_start_zero,q_first_wp,traj_options);

q_traj = [q_init(:,1:end-1),q_traj];
qd_traj = [qd_init(:,1:end-1),qd_traj];
qdd_traj = [qdd_init(:,1:end-1),qdd_traj];
t_vec = [t_init(1:end-1),t_vec+t_init(end)];

numSamples = 241;
segment_length = numSamples-1;
nTotal = size(q_traj,2);
nSegments = round((nTotal-1)/segment_length);

if nSegments*segment_length+1 ~= nTotal
    error('Trajectory length is inconsistent with numSamples=241.');
end

%% ========================================================================
% DISCRETE MODES
% =========================================================================

tile_states = buildTileStates_local(7);
nModes = numel(tile_states);
if nModes ~= 15
    error('Expected 15 discrete configurations, found %d.',nModes);
end

open(modelFile);

%% ========================================================================
% INITIAL CONTROLLER
% =========================================================================

load(initialControllerFile,'K_10DOF_Tuned');
if ~exist('K_10DOF_Tuned','var') || ~isequal(size(K_10DOF_Tuned),[10 30])
    error('K_10DOF_Tuned must be a 10x30 numeric matrix.');
end

%% ========================================================================
% COMMON SCHEDULING GRID FOR THE ANALYTIC SURFACE
% =========================================================================

sGrid = linspace(0,1,Nfit);

% 45 analytic branches: 15 modes x 3 segments in each local mode window.
branchMode = [];
branchSegment = [];
branchS = [];

branchPlant = {};
branchIndex = 0;

fprintf('\n==============================================================\n');
fprintf('ANALYTIC / PIECEWISE LPV GENERATION\n');
fprintf('Source      : %s\n',ANALYTIC_SOURCE);
fprintf('Trajectory P: %d\n',P);
fprintf('Plant P     : %d\n',Pplant);
fprintf('Branches    : 15 modes x local 3-segment windows\n');
fprintf('==============================================================\n');

for m = 1:nModes

    [seg_start,seg_end] = localWindowForMode_local(m);

    % The local mode window contains up to 3 actual quintic segments.
    % We create one analytic branch per actual segment.
    for seg = seg_start:seg_end-1

        branchIndex = branchIndex+1;
        branchMode(branchIndex,1) = m;
        branchSegment(branchIndex,1) = seg;
        branchS(:,branchIndex) = sGrid(:);

        idx_start = seg*segment_length+1;
        idx_end = (seg+1)*segment_length+1;

        if idx_start < 1 || idx_end > nTotal
            error('Mode %d, segment %d maps outside trajectory.',m,seg);
        end

        q0 = q_traj(:,idx_start);
        qf = q_traj(:,idx_end);

        % ================================================================
        % EXACT QUINTIC TRAJECTORY ON THIS BRANCH
        % q(s) = q0 + (qf-q0)*(10s^3-15s^4+6s^5)
        % ================================================================
        coeff_q_exact = exactQuinticCoefficients(q0,qf);

        % Keep a numerical validation of the polynomial against the stored
        % trajectory.
        qfit = zeros(nArm,Nfit);
        for j = 1:nArm
            qfit(j,:) = polyval(coeff_q_exact(j,:),sGrid);
        end

        qdata = q_traj(:,idx_start:idx_end);
        sdata = linspace(0,1,size(qdata,2));
        qcheck = interp1(sdata,qdata.',sGrid,'linear').';
        fitError = max(abs(qfit(:)-qcheck(:)));

        if fitError > 1e-8
            fprintf('Warning mode %d seg %d: stored trajectory vs exact quintic max error %.3e\n',...
                m,seg,fitError);
        end

        switch upper(ANALYTIC_SOURCE)
            case 'PARAMETRIC'
                % ---------------------------------------------------------
                % TRUE DOCTORAL-STYLE ROUTE.
                % The returned model must already expose q-dependent
                % linearized quantities through ureal/uss parameters.
                % ---------------------------------------------------------
                Gbranch = getParametricPlantMode_local( ...
                    m,seg,branchMode,branchSegment,sGrid,...
                    q0,qf,coeff_q_exact,modelName, ...
                    TilePlacementVector_local(tile_states(m)));

            case 'FIT_LTI'
                % ---------------------------------------------------------
                % Runnable fallback using current ulinearize workflow.
                % Fit A,B,C,D vs local s with polynomial degree Pplant.
                % ---------------------------------------------------------
                Gbranch = fitPlantFromFrozenLinearizations_local(...
                    m,seg,q_traj,qd_traj,qdd_traj,...
                    idx_start,idx_end,Nfit,Pplant,...
                    tile_states(m),modelName,all_inputs,all_outputs);

            otherwise
                error('Unknown ANALYTIC_SOURCE=%s.',ANALYTIC_SOURCE);
        end

        branchPlant{branchIndex} = Gbranch;

        fprintf('Branch %2d: mode=%2d, segment=%2d, fit/max model built.\n',...
            branchIndex,m,seg);
    end
end

nBranches = branchIndex;

% Convert branch list into a model array. The branch variable(s) are discrete
% scenario labels. The continuous parameter inside each branch is s.
G_analytic = stack(1,branchPlant{:});

% The true parametric path has a single local s for each branch; use a
% SamplingGrid that records the branch information.
Sbranch = zeros(nBranches,1);
for b = 1:nBranches
    % The tunableSurface below uses the common normalized phase s.
    % For an uncertain-parameter plant, the actual ureal 's' is the active
    % scheduling parameter. Keep the branch labels here for bookkeeping.
    Sbranch(b) = 0.5;
end
G_analytic.SamplingGrid = struct(...
    'mode',branchMode,...
    'segment',branchSegment);

%% ========================================================================
% ANALYTIC CONTROLLER: K(s) = K0 + s*KL*KR
% =========================================================================

K0 = K_10DOF_Tuned;

KL = tunableGain('KL',nDOF,rankK);
KR = tunableGain('KR',rankK,nPID);

% The doctoral script initializes the variation without prescribing it.
KL.Gain.Value = zeros(nDOF,rankK);
KR.Gain.Value = zeros(rankK,nPID);

s = ureal('s',0.5,'Range',[0 1]);

K_analytic = tunableGain('K0',K0) + s*KL*KR;
K_analytic.InputName = pid_inputs;
K_analytic.OutputName = u;

%% ========================================================================
% NOTE ABOUT PARAMETRIC TUNING
% =========================================================================
% For the TRUE PARAMETRIC route, every branch model must contain the same
% scheduling parameter named 's'. This is what makes the controller vary
% with the same scalar parameter as the plant.
%
% For FIT_LTI, G_analytic is a polynomial surrogate built from frozen
% nominal LTI models. It is intended as an approximation study.

G_full = G_analytic(all_outputs,all_inputs);
CL_analytic = lft(G_full,K_analytic);

%% ========================================================================
% REQUIREMENTS -- IDENTICAL TO High_Control.m
% =========================================================================

[Soft_Goals,Hard_Goals] = buildGoals_local(...
    ref_aocs,ref_arm,disturb_aocs,disturb_arm,...
    q_aocs,q_arm,err_aocs,err_arm,torque_aocs,torque_arm);

%% ========================================================================
% TUNING
% =========================================================================

if RUN_TUNING

    opt = systuneOptions('MaxIter',500,...
                         'RandomStart',1,...
                         'UseParallel',true,...
                         'SoftTarget',1200,...
                         'SoftScale',1700,...
                         'SoftTol',0.07,...
                         'Display','iter');

    fprintf('\nStarting analytic tuning...\n');
    [CL_analytic_Tuned,fSoft,gHard,Info] = systune(...
        CL_analytic,Soft_Goals,Hard_Goals,opt); %#ok<ASGLU>

    fprintf('\n===== ANALYTIC LPV RESULTS =====\n');
    fprintf('Hard goal peak : %.6f\n',gHard);
    fprintf('Soft goal      : %.6f\n',fSoft);

    K0_tuned = getBlockValue(CL_analytic_Tuned,'K0');
    KL_tuned = getBlockValue(CL_analytic_Tuned,'KL');
    KR_tuned = getBlockValue(CL_analytic_Tuned,'KR');

    if SAVE_RESULTS
        save('LPV_analytic_piecewise_result.mat',...
            'CL_analytic_Tuned','K0_tuned','KL_tuned','KR_tuned',...
            'fSoft','gHard','Info','branchMode','branchSegment','P','Pplant');
    end
else
    fprintf('\nRUN_TUNING=false: analytic model/controller created only.\n');
end

fprintf('\nDone.\n');

%% ========================================================================
% LOCAL FUNCTIONS
% =========================================================================

function coeff = exactQuinticCoefficients(q0,qf)
% Exact coefficients in descending MATLAB polyfit/polyval order:
% q(s) = q0 + dq*(10*s^3 - 15*s^4 + 6*s^5)
    dq = qf-q0;
    n = numel(q0);
    coeff = zeros(n,6);
    coeff(:,1) = 6*dq;      % s^5
    coeff(:,2) = -15*dq;    % s^4
    coeff(:,3) = 10*dq;     % s^3
    coeff(:,4) = 0;         % s^2
    coeff(:,5) = 0;         % s
    coeff(:,6) = q0;        % constant
end

function Gbranch = fitPlantFromFrozenLinearizations_local(...
        m,seg,q_traj,qd_traj,qdd_traj,idx_start,idx_end,...
        Nfit,Pplant,tileState,modelName,all_inputs,all_outputs)
% Fallback analytic surrogate.
% Each A/B/C/D entry is fitted as a polynomial in local s.
%
% WARNING: This uses the nominal frozen state-space matrices. It does not
% preserve the original USS uncertainty blocks.

    nTotalLocal = idx_end-idx_start+1;
    sFit = linspace(0,1,Nfit);
    idxFit = round(1 + sFit*(nTotalLocal-1));

    Astack = cell(1,Nfit);
    Bstack = cell(1,Nfit);
    Cstack = cell(1,Nfit);
    Dstack = cell(1,Nfit);

    for k = 1:Nfit

        localIdx = idxFit(k);
        q_i = q_traj(:,idx_start+localIdx-1); %#ok<NASGU>
        qd_i = qd_traj(:,idx_start+localIdx-1); %#ok<NASGU>
        qdd_i = qdd_traj(:,idx_start+localIdx-1); %#ok<NASGU>

        placements = tileState.placements;
        Tile1_Placement = placements(1); %#ok<NASGU>
        Tile2_Placement = placements(2); %#ok<NASGU>
        Tile3_Placement = placements(3); %#ok<NASGU>
        Tile4_Placement = placements(4); %#ok<NASGU>
        Tile5_Placement = placements(5); %#ok<NASGU>
        Tile6_Placement = placements(6); %#ok<NASGU>
        Tile7_Placement = placements(7); %#ok<NASGU>

        Data_sat_Nominal;

        sys = ulinearize(modelName);
        sys = minreal(sys);

        % For this surrogate only the nominal plant matrices are fitted.
        sysNom = getNominal(sys);

        Astack{k} = sysNom.A;
        Bstack{k} = sysNom.B;
        Cstack{k} = sysNom.C;
        Dstack{k} = sysNom.D;
    end

    nX = size(Astack{1},1);
    nU = size(Bstack{1},2);
    nY = size(Cstack{1},1);

    % The SAME real parameter name is used in every branch. This is essential:
    % the tuned controller K(s) and every plant branch must share the same
    % scheduling variable.
    s = ureal('s',0.5,'Range',[0 1]);

    A = zeros(nX,nX);
    B = zeros(nX,nU);
    C = zeros(nY,nX);
    D = zeros(nY,nU);

    % Polynomial coefficients in ascending powers for direct evaluation.
    for i = 1:nX
        for j = 1:nX
            y = zeros(1,Nfit);
            for k = 1:Nfit
                y(k) = Astack{k}(i,j);
            end
            A(i,j) = polyval(polyfit(sFit,y,Pplant),s);
        end
    end

    for i = 1:nX
        for j = 1:nU
            y = zeros(1,Nfit);
            for k = 1:Nfit
                y(k) = Bstack{k}(i,j);
            end
            B(i,j) = polyval(polyfit(sFit,y,Pplant),s);
        end
    end

    for i = 1:nY
        for j = 1:nX
            y = zeros(1,Nfit);
            for k = 1:Nfit
                y(k) = Cstack{k}(i,j);
            end
            C(i,j) = polyval(polyfit(sFit,y,Pplant),s);
        end
    end

    for i = 1:nY
        for j = 1:nU
            y = zeros(1,Nfit);
            for k = 1:Nfit
                y(k) = Dstack{k}(i,j);
            end
            D(i,j) = polyval(polyfit(sFit,y,Pplant),s);
        end
    end

    Gbranch = ss(A,B,C,D);
    Gbranch.u = all_inputs;
    Gbranch.y = all_outputs;
end

function Gbranch = getParametricPlantMode_local(...
        m,seg,branchMode,branchSegment,sGrid,q0,qf,coeff_q,...
        modelName,placements)
% ================================================================
% MODEL-SPECIFIC ADAPTER FOR THE TRUE DOCTORAL-STYLE METHOD.
% ================================================================
%
% Your doctoral reference has uncertain parameters such as
%   tan_theta1_div4, ...
% and then does
%   usubs(M,'tan_theta1_div4',theta1_traj,...)
%
% To replicate that architecture, your SDT linearized model must expose
% the configuration-dependent linearized quantities as ureal/uss blocks.
%
% This function is intentionally isolated because those parameter names and
% their relation to the Simulink linearization are model-specific.
%
% EXPECTED CONTRACT:
%   return a USS/GSS model containing a common uncertain real scheduling
%   parameter named 's' with Range [0,1], and where q-dependent model
%   parameters have been substituted by the exact piecewise quintic q(s).
%
% Do NOT replace this by ulinearize() at one operating point: that would
% destroy the continuous parameter dependence.

    %#ok<INUSD>
    error([ ...
        'TRUE PARAMETRIC mode requires a parameterized linear plant.\n' ...
        'Implement getParametricPlantMode_local() for your SDT model.\n' ...
        'The current ulinearize() script only returns frozen LTI/USS models;\n' ...
        'it does not expose A(q), B(q), ... as functions of q.\n\n' ...
        'Use the FIT_LTI mode to run a complete numerical surrogate now, or\n' ...
        'show me the uncertainty/parameter blocks produced by your ulinearize\n' ...
        'model and this adapter can be made exact.']);
end

function v = TilePlacementVector_local(tileState)
    v = tileState.placements(:).';
end

function [seg_start,seg_end] = localWindowForMode_local(i)
    if i == 1
        seg_start = 0;
        seg_end = 2;
    elseif i == 15
        seg_start = 3*i - 4;
        seg_end = 42;
    else
        seg_start = 3*i - 4;
        seg_end = 3*i - 1;
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

function [Soft_Goals,Hard_Goals] = buildGoals_local(...
        ref_aocs,ref_arm,disturb_aocs,disturb_arm,...
        q_aocs,q_arm,err_aocs,err_arm,torque_aocs,torque_arm)

    wm_arm = 0.1;
    wt_aocs = 0.01;
    step_max_arm = 3.11;
    step_max_aocs = deg2rad(3.0);
    tol_base_wobble = deg2rad(3.0);
    tol_base_fine = deg2rad(0.5);
    tol_arm_deflect = 0.015/5.15;
    max_torque_arm = 1500.0;
    max_torque_aocs = 0.82;
    tau_env_max = 1e-3;
    Fc = 0.28;
    Fs = 0.34;
    v_s = 0.1;

    W_Sens_arm = makeweight(0.01,wm_arm,2.0);
    Req_Sens_arm = TuningGoal.Gain(ref_arm,err_arm,W_Sens_arm);
    W_Sens_aocs = makeweight(0.01,wt_aocs,2.0);
    Req_Sens_aocs = TuningGoal.Gain(ref_aocs,err_aocs,W_Sens_aocs);

    gain_dc_arm2base = tol_base_fine/step_max_arm;
    gain_peak_arm2base = tol_base_wobble/step_max_arm;
    W_Sens_arm2base = makeweight(...
        gain_dc_arm2base,...
        [wm_arm*0.1,(gain_dc_arm2base+gain_peak_arm2base)/2],...
        gain_peak_arm2base);
    Req_Sens_arm2base = TuningGoal.Gain(ref_arm,q_aocs,W_Sens_arm2base);

    gain_dc_base2arm = tol_arm_deflect/step_max_aocs;
    gain_peak_base2arm = (tol_arm_deflect*10)/step_max_aocs;
    W_Sens_base2arm = makeweight(...
        gain_dc_base2arm,...
        [wt_aocs*3,(gain_dc_base2arm+gain_peak_base2arm)/2],...
        gain_peak_base2arm);
    Req_Sens_base2arm = TuningGoal.Gain(ref_aocs,q_arm,W_Sens_base2arm);

    Gain_Limit_aocs = max_torque_aocs/step_max_aocs;
    Gain_Limit_arm = max_torque_arm/step_max_arm;
    Req_Effort_aocs = TuningGoal.Gain(ref_aocs,torque_aocs,Gain_Limit_aocs);
    Req_Effort_arm = TuningGoal.Gain(ref_arm,torque_arm,Gain_Limit_arm);

    Gain_Limit_aocs_cross = max_torque_aocs/step_max_arm;
    Gain_Limit_arm_cross = max_torque_arm/step_max_aocs;
    Req_Effort_arm2base = TuningGoal.Gain(ref_arm,torque_aocs,Gain_Limit_aocs_cross);
    Req_Effort_base2arm = TuningGoal.Gain(ref_aocs,torque_arm,Gain_Limit_arm_cross);

    compliance_env = tol_base_fine/tau_env_max;
    Req_Dist_AOCS = TuningGoal.Gain(...
        disturb_aocs,q_aocs,...
        makeweight(compliance_env,[wt_aocs/3,compliance_env*5],compliance_env*10));

    M_f = @(v) Fc + (Fs-Fc)./(1+(v./v_s).^2);
    W_dist_arm_friction = makeweight(...
        tol_arm_deflect/M_f(0),...
        [wm_arm/2,tol_arm_deflect/M_f(0.05)],...
        tol_arm_deflect/M_f(0.1));
    Req_Dist_Arm_Friction = TuningGoal.Gain(...
        disturb_arm,q_arm,W_dist_arm_friction);

    Hard_Goals = [Req_Sens_arm,Req_Sens_aocs,Req_Sens_base2arm,Req_Sens_arm2base,...
                  Req_Effort_aocs,Req_Effort_arm,Req_Effort_base2arm,...
                  Req_Dist_Arm_Friction,Req_Dist_AOCS];
    Soft_Goals = Req_Effort_arm2base;
end

function [t,q,qd,qdd] = trajectoryGeneration_local(q_start,q_goal,options)
    T = options.totalTime;
    n_samples = options.numSamples;
    t = linspace(0,T,n_samples);
    tau = t/T;
    s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
    s_dot = (30*tau.^2 - 60*tau.^3 + 30*tau.^4)/T;
    s_ddot = (60*tau - 180*tau.^2 + 120*tau.^3)/(T^2);
    delta_q = q_goal-q_start;
    q = q_start + delta_q*s;
    qd = delta_q*s_dot;
    qdd = delta_q*s_ddot;
end
