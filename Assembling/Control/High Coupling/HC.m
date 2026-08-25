%% ========================================================================
%  HIGHLY COUPLED TUNING FOR SPACE MANIPULATOR (10-DOF MIMO PID)
%  System: 7-DOF Robotic Arm + 3-Axis Satellite Base (ADCS/AOCS)
%  Approach: Monolithic Full-State / Cross-Coupled Control
%  ========================================================================
clc; clear; close all; 

fprintf('\n=== STARTING HIGHLY COUPLED 10-DOF CONTROLLER TUNING ===\n');

%% PHASE 0: Define Signal Names matching Simulink Root Ports
gen_names = @(prefix, n) arrayfun(@(x) sprintf('%s(%d)', prefix, x), 1:n, 'UniformOutput', false);

% --- EXOGENOUS INPUTS (w) ---
ref_aocs     = gen_names('Ref_AOCS', 3);
ref_arm      = gen_names('Ref_Arm', 7);
disturb_aocs = gen_names('Disturb_AOCS', 3);
disturb_arm  = gen_names('Disturb_Arm', 7);

% --- CONTROL INPUTS (u) - Must be last for LFT ---
u  = gen_names('u', 10);

all_inputs = [ref_aocs, ref_arm, disturb_aocs, disturb_arm, u];

% --- EXOGENOUS OUTPUTS (z) - Exposed for Performance Evaluation ---
q_aocs           = gen_names('q_AOCS', 3);
q_arm            = gen_names('q_Arm', 7);
err_aocs         = gen_names('Err_AOCS', 3);  
err_arm          = gen_names('Err_Arm', 7);
torque_aocs      = gen_names('Torque_AOCS', 3);
torque_arm       = gen_names('Torque_Arm', 7);

% --- MEASUREMENT OUTPUTS (v) - Consumed by LFT ---
pid_inputs = gen_names('pid_inputs', 30);   % 30x1 vector (q, int_e, v)

all_outputs = [q_aocs, q_arm, err_aocs, err_arm, torque_aocs, torque_arm, ...
               pid_inputs];

%% PART 0: Extract Uncertain Plant Array
% sdtModel = 'SDT_Dynamics';
open('SDT_Control_Tuning_HC.slx');
tile_states  = buildTileStates(7);
n_samples    = numel(tile_states);
traj_indices = round(linspace(1, size(q_traj, 2), n_samples));
q_samples    = q_traj(:, traj_indices);

G_SDT_samples = cell(n_samples, 1);
fprintf('Extracting Uncertain Open-Loop Models...\n');

for i = 1:n_samples
    q_i = q_samples(:, i);
    placements = tile_states(i).placements;
    Tile1_Placement = placements(1);
    Tile2_Placement = placements(2);
    Tile3_Placement = placements(3);
    Tile4_Placement = placements(4);
    Tile5_Placement = placements(5);
    Tile6_Placement = placements(6);
    Tile7_Placement = placements(7);
    
    sys_open_loop = ulinearize('SDT_Control_Tuning_HC');
    sys_open_loop = minreal(sys_open_loop);

    % Force the I/O names to match string arrays based on Simulink port order
    sys_open_loop.u  = all_inputs;
    sys_open_loop.y = all_outputs;
    
    G_SDT_samples{i} = sys_open_loop;
end

% Stack into a Multi-Model Uncertain State-Space Array
G_array = stack(1, G_SDT_samples{:});

%% 1. MISSION SPECIFICATIONS & OPERATIONAL BOUNDS
% -------------------------------------------------------------------------
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


%% 2. PLANT I/O CONCATENATION (10x10)

% Extract the fully coupled 10-DOF plant
G_full = G_array(all_outputs, all_inputs);
n_dof  = 10; % 7 Arm joints + 3 Satellite axes

%% 3. HYBRID CONTROLLER CONSTRUCTION
% -------------------------------------------------------------------------
% Proportional Gain: Full 10x10 matrix (allows immediate dynamic decoupling)
Kp_Full = tunableGain('Kp_Full', eye(n_dof)); 
Kp_Full.Gain.Free = true(n_dof, n_dof);  
Kp_Full.Gain.Value = blkdiag(K_aocs_tuned.D(1:3,1:3), K_arm_tuned.D(1:7,1:7));

% Integral Gain: Block-Diagonal (7x7 Arm, 3x3 Base) to prevent Cross-Windup
Ki_Full = tunableGain('Ki_Full', eye(n_dof)); 
Ki_Full.Gain.Free = blkdiag(true(3,3), true(7,7));
Ki_Full.Gain.Value = blkdiag(K_aocs_tuned.D(1:3,4:6), K_arm_tuned.D(1:7,8:14));


% Derivative Gain: Full 10x10 matrix (compensates Coriolis & inertia cross-terms)
Kd_Full = tunableGain('Kd_Full', eye(n_dof)); 
Kd_Full.Gain.Free = true(n_dof, n_dof);
Kd_Full.Gain.Value = blkdiag(K_aocs_tuned.D(1:3,7:9), K_arm_tuned.D(1:7,15:21));


% Concatenate into a unified monolithic PID [10 outputs x 30 inputs]
% Structure: [ Kp | Ki | Kd ] (Match your specific feedback sign convention)
K_Full_Matrix = [Kp_Full, Ki_Full, Kd_Full];
K_Full_Matrix.InputName  = pid_inputs; 
K_Full_Matrix.OutputName = u;

% Close the MIMO feedback loop
CL_Full_Coupled = lft(G_full, K_Full_Matrix);

%% 4. MIMO TUNING GOALS FORMULATION

% --- GOAL 1: MIMO Sensitivity S(s) & Asymmetric Decoupling ---
% Diagonal tracking profiles
W_Sens_arm  = makeweight(0.01, wm_arm,  2.0);
Req_Sens_arm = TuningGoal.Gain(ref_arm, err_arm, W_Sens_arm);
W_Sens_aocs = makeweight(0.01, wt_aocs, 2.0);
Req_Sens_aocs = TuningGoal.Gain(ref_aocs, err_aocs, W_Sens_aocs);

% Off-Diagonal A (Arm Motion -> Base Disturbance)
gain_dc_arm2base   = tol_base_fine   / step_max_arm; % 0.5 deg norm
gain_peak_arm2base = tol_base_wobble / step_max_arm; % 3.0 deg norm
W_Sens_arm2base = makeweight(gain_dc_arm2base, [wm_arm*0.1, (gain_dc_arm2base+gain_peak_arm2base)/2] , gain_peak_arm2base);

% Off-Diagonal B (Wide Base Slew -> Arm Disturbance)
gain_dc_base2arm   = tol_arm_deflect / step_max_aocs;
gain_peak_base2arm = (tol_arm_deflect*10) / step_max_aocs;
W_Sens_base2arm = makeweight(gain_dc_base2arm, [wt_aocs*3,(gain_dc_base2arm+gain_peak_base2arm)/2], gain_peak_base2arm);       %da aggiustare

%Req_Sens_MIMO = TuningGoal.Gain(ref_all, err_all, W_Sens_MIMO);
Req_Sens_base2arm = TuningGoal.Gain(ref_aocs, q_arm, W_Sens_base2arm);
Req_Sens_arm2base = TuningGoal.Gain(ref_arm, q_aocs, W_Sens_arm2base);



% --- GOAL 2: Actuator Effort & RW Cross-Saturation Protection ---
%Gain_Limit = zeros(n_dof, n_dof);
Gain_Limit_aocs   = max_torque_aocs  / step_max_aocs;
Gain_Limit_arm = max_torque_arm / step_max_arm;

Req_Effort_aocs = TuningGoal.Gain(ref_aocs, torque_aocs, Gain_Limit_aocs);
Req_Effort_arm = TuningGoal.Gain(ref_arm, torque_arm, Gain_Limit_arm);


% Cross-Limits: Arm motion must NEVER request > 0.82 Nm from Reaction Wheels
Gain_Limit_aocs_cross  = max_torque_aocs / step_max_arm;
Gain_Limit_arm_cross  = max_torque_arm  / step_max_aocs;

Req_Effort_arm2base = TuningGoal.Gain(ref_arm, torque_aocs, Gain_Limit_aocs_cross);
Req_Effort_base2arm = TuningGoal.Gain(ref_aocs, torque_arm, Gain_Limit_arm_cross);


% --- GOAL 3: External & Environmental Disturbance Rejection ---

% Satellite LEO environmental disturbance rejection
compliance_env = tol_base_fine / tau_env_max;
Req_Dist_AOCS  = TuningGoal.Gain(disturb_aocs, q_aocs, makeweight(compliance_env, [wt_aocs/3, compliance_env*5],  compliance_env*10));

% Joints Friction
M_f = @(v) Fc + (Fs-Fc)*1./(1+(v./v_s).^2);
W_dist_arm_friction = makeweight(tol_arm_deflect/M_f(0),[wm_arm/2, tol_arm_deflect/M_f(0.05)], tol_arm_deflect/M_f(0.1));
Req_Dist_Arm_Friction = TuningGoal.Gain(disturb_arm, q_arm, W_dist_arm_friction);


%% 5. SYSTUNE OPTIMIZATION
% -------------------------------------------------------------------------
% Hard Goals: Strict stability, cross-decoupling, and actuator limits
Hard_Goals = [Req_Sens_arm, Req_Sens_aocs, Req_Sens_base2arm, ...
    Req_Effort_aocs, Req_Effort_arm, Req_Effort_base2arm, ...
    Req_Dist_Arm_Friction, Req_Dist_AOCS];

 %Hard_Goals = [Req_Sens_arm, Req_Sens_aocs, Req_Sens_arm2base, Req_Sens_base2arm];

% Soft Goals: Minimize disturbance response as much as physically possible
Soft_Goals = [Req_Sens_arm2base,Req_Effort_arm2base];

% Solver setup: Increased MaxIter due to high parameter dimensionality (258 free vars)
opt = systuneOptions('MaxIter', 50, ...
                     'RandomStart', 0, ...
                     'UseParallel', false, ...
                     'Display', 'iter');

fprintf('\nLaunching solver... (Searching 258 free parameters across 10 DOF)\n');
[CL_MIMO_Tuned, fSoft, gHard] = systune(CL_Full_Coupled, Soft_Goals, Hard_Goals, opt)

%% 6. EXTRACT, SAVE AND VERIFY GAINS
% -------------------------------------------------------------------------
fprintf('\n--- TUNING RESULTS ---');
fprintf('\nHard Goal Peak Value (gHard): %.4f (Must be <= 1.0 for feasibility)', gHard);
fprintf('\nSoft Goal Value      (fSoft): %.4f\n', fSoft);

% Extract optimized blocks
Kp_opt = getBlockValue(CL_MIMO_Tuned, 'Kp_Full');
Ki_opt = getBlockValue(CL_MIMO_Tuned, 'Ki_Full');
Kd_opt = getBlockValue(CL_MIMO_Tuned, 'Kd_Full');

% Unified 10x30 Controller Matrix
K_10DOF_Tuned = [Kp_opt, Ki_opt, Kd_opt];

save('K_10DOF_HighlyCoupled2.mat', 'K_10DOF_Tuned');
fprintf('Successfully saved controller to "K_10DOF_HighlyCoupled.mat".\n');

% --- Visual Verification ---
figure('Name', 'MIMO Sensitivity & Cross-Talk');
viewGoal(Req_MIMO_Sens, CL_MIMO_Tuned);

figure('Name', 'Control Effort & RW Protection');
viewGoal(Req_MIMO_Effort, CL_MIMO_Tuned);

figure('Name', 'Environmental Disturbance Rejection');
subplot(2,1,1); viewGoal(Req_Dist_Arm, CL_MIMO_Tuned); title('Arm Tip Disturbance');
subplot(2,1,2); viewGoal(Req_Dist_AOCS, CL_MIMO_Tuned); title('Base Environmental Disturbance');

%% FUNCTIONS
 function tile_states = buildTileStates(n_tiles)
% buildTileStates returns a sequential placement schedule with a single tile on the arm.
% State 1 = start, State 2 = on end-effector, State 3 = placed on antenna.

states = struct('name', {}, 'placements', {});
idx = 1;

states(idx).name = 'Start';
states(idx).placements = ones(1, n_tiles);
idx = idx + 1;

for k = 1:n_tiles
    placements = ones(1, n_tiles);
    if k > 1
        placements(1:k-1) = 3;
    end
    
    placements(k) = 2;
    states(idx).name = sprintf('Tile %d Grab', k);
    states(idx).placements = placements;
    idx = idx + 1;
    
    placements(k) = 3;
    states(idx).name = sprintf('Tile %d Placed', k);
    states(idx).placements = placements;
    idx = idx + 1;
end
tile_states = states;
end

