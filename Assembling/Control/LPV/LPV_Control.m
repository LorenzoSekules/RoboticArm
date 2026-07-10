%% ========================================================================
%  HIGHLY COUPLED LPV TUNING FOR SPACE MANIPULATOR (10-DOF MIMO PID)
%  System: 7-DOF Robotic Arm + 3-Axis Satellite Base (ADCS/AOCS)
%  Approach: 1D Discrete LPV Tunable Surface over Assembly Stages S in [1, 15]
%  ========================================================================
clc; clear; close all;
fprintf('\n=== STARTING HIGHLY COUPLED 10-DOF LPV CONTROLLER TUNING ===\n');

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

%% PART 0: Extract Gridded Plant Array across Assembly Stages
open('SDT_Control_Tuning_HC.slx');
tile_states  = buildTileStates(7);
n_samples    = numel(tile_states);
traj_indices = round(linspace(1, size(q_traj, 2), n_samples));
q_samples    = q_traj(:, traj_indices);

G_SDT_samples = cell(n_samples, 1);
fprintf('Extracting Open-Loop Models across %d Assembly Stages...\n', n_samples);

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
    sys_open_loop.u = all_inputs;
    sys_open_loop.y = all_outputs;
    
    G_SDT_samples{i} = sys_open_loop;
end

% [UPGRADE 1]: Stack along dimension 3 to create a 3D LTI Array (Outputs x Inputs x Stages)
G_array = stack(2, G_SDT_samples{:});

% Attach formal Sampling Grid to tie models to discrete stage parameter S
StageGrid = 1:n_samples;
G_array.SamplingGrid = struct('S', StageGrid);

%% 1. MISSION SPECIFICATIONS & OPERATIONAL BOUNDS
% -------------------------------------------------------------------------
wm_arm  = 0.1;          % [rad/s] Manipulator joint bandwidth
wt_aocs = 0.1;          % [rad/s] Satellite base attitude slew bandwidth

step_max_arm  = 3.11;           % [rad] Full extension reach maneuver
step_max_aocs = deg2rad(15.0);  % [rad] Max wide slew

tol_base_wobble = deg2rad(3.0); % [rad] Max transient base tilt during arm slew
tol_base_fine   = deg2rad(0.5); % [rad] Max steady-state base tilt when arm stops
tol_arm_deflect = 0.015/5.15; % [rad] Max arm joint movement during satellite slew

max_torque_arm  = 1500.0; % [Nm] Joint drive maximum capability
max_torque_aocs = 0.82;   % [Nm] Reaction Wheel (RW) maximum torque

tau_env_max   = 1e-3;   % [Nm] LEO Orbital disturbances
arm_length    = 5.15;   % [m] Total manipulator span
tip_disp_max  = 0.015;  % [m] Maximum structural tip compliance threshold

%% 2. PLANT I/O CONCATENATION (10x10)
G_full = G_array(all_outputs, all_inputs);
n_dof  = 10; % 7 Arm joints + 3 Satellite axes

%% 3. LPV TUNABLE SURFACE CONTROLLER CONSTRUCTION
% -------------------------------------------------------------------------
% [UPGRADE 2]: Define the Scheduling Domain and Shape Function over Stage S
domain = struct('S', StageGrid);

% Quadratic polynomial basis functions over normalized stage domain [-1, 1]
shapeFcn = @(x) [x, x^2]; 
num_fct = 3;

% Extract initial baseline gain values (from existing nominal tuning if available)
Kp_init = blkdiag(K_aocs_tuned.D(1:3,1:3), K_arm_tuned.D(1:7,1:7));
Ki_init = blkdiag(K_aocs_tuned.D(1:3,4:6), K_arm_tuned.D(1:7,8:14));
Kd_init = blkdiag(K_aocs_tuned.D(1:3,7:9), K_arm_tuned.D(1:7,15:21));

% Proportional Gain Surface: Full 10x10 dynamic decoupling allowed
Kp_Surface = tunableSurface('Kp_Full', Kp_init, domain, shapeFcn); 
Kp_Surface.Coefficients.Free = true(n_dof, num_fct*n_dof);  

% Proportional Gain Surface: Full 10x10 dynamic decoupling allowed
Ki_Surface = tunableSurface('Ki_Full', Ki_init, domain, shapeFcn); 
Ki_Surface.Coefficients.Free = [blkdiag(true(3,3), true(7,7)),blkdiag(true(3,3), true(7,7)),blkdiag(true(3,3), true(7,7))];  


% Derivative Gain Surface: Full 10x10 matrix to compensate Coriolis & inertia shifts
Kd_Surface = tunableSurface('Kd_Full', Kd_init, domain, shapeFcn); 
Kd_Surface.Coefficients.Free = true(n_dof, num_fct*n_dof);

% Concatenate into unified LPV PID Surface [10 outputs x 30 inputs]
K_LPV_Surface = [Kp_Surface, Ki_Surface, Kd_Surface];


% Close the MIMO LPV feedback loop across all grid stages
CL_Full_Coupled = lft(G_full, K_LPV_Surface);

%% 4. MIMO TUNING GOALS FORMULATION
% -------------------------------------------------------------------------
% Diagonal tracking profiles
W_Sens_arm    = makeweight(0.01, wm_arm, 2.0);
Req_Sens_arm  = TuningGoal.Gain(ref_arm, err_arm, W_Sens_arm);

W_Sens_aocs   = makeweight(0.01, wt_aocs, 2.0);
Req_Sens_aocs = TuningGoal.Gain(ref_aocs, err_aocs, W_Sens_aocs);

% Off-Diagonal A (Arm Motion -> Base Disturbance)
gain_dc_arm2base   = tol_base_fine   / step_max_arm; 
gain_peak_arm2base = tol_base_wobble / step_max_arm; 
W_Sens_arm2base    = makeweight(gain_dc_arm2base, [wm_arm*0.1, (gain_dc_arm2base+gain_peak_arm2base)/2], gain_peak_arm2base);
Req_Sens_arm2base  = TuningGoal.Gain(ref_arm, q_aocs, W_Sens_arm2base);

% Off-Diagonal B (Wide Base Slew -> Arm Disturbance)
gain_dc_base2arm   = tol_arm_deflect / step_max_aocs;
gain_peak_base2arm = (tol_arm_deflect*10) / step_max_aocs;
W_Sens_base2arm    = makeweight(gain_dc_base2arm, [wt_aocs*3, (gain_dc_base2arm+gain_peak_base2arm)/2], gain_peak_base2arm);
Req_Sens_base2arm  = TuningGoal.Gain(ref_aocs, q_arm, W_Sens_base2arm);

% Actuator Effort & RW Cross-Saturation Protection
Gain_Limit_aocs     = max_torque_aocs  / step_max_aocs;
Gain_Limit_arm      = max_torque_arm / step_max_arm;
Req_Effort_aocs     = TuningGoal.Gain(ref_aocs, torque_aocs, Gain_Limit_aocs);
Req_Effort_arm      = TuningGoal.Gain(ref_arm, torque_arm, Gain_Limit_arm);

Gain_Limit_aocs_cross = max_torque_aocs / step_max_arm;
Gain_Limit_arm_cross  = max_torque_arm  / step_max_aocs;
Req_Effort_arm2base   = TuningGoal.Gain(ref_arm, torque_aocs, Gain_Limit_aocs_cross);
Req_Effort_base2arm   = TuningGoal.Gain(ref_aocs, torque_arm, Gain_Limit_arm_cross);

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
Hard_Goals = [Req_Sens_arm, Req_Sens_aocs, Req_Sens_arm2base, Req_Sens_base2arm, ...
              Req_Effort_aocs, Req_Effort_arm, Req_Effort_arm2base, Req_Effort_base2arm, ...
              Req_Dist_Arm, Req_Dist_AOCS];
Soft_Goals = [];

opt = systuneOptions('MaxIter', 500, ...
                     'RandomStart', 5, ...
                     'UseParallel', true, ...
                     'Display', 'iter');

fprintf('\nLaunching LPV Surface Solver across %d Assembly Stages...\n', n_samples);
[CL_MIMO_Tuned, fSoft, gHard] = systune(CL_Full_Coupled, Soft_Goals, Hard_Goals, opt);

%% 6. EXTRACT, SAVE AND VERIFY LPV GAIN SURFACES
% -------------------------------------------------------------------------
fprintf('\n--- LPV TUNING RESULTS ---');
fprintf('\nHard Goal Peak Value (gHard): %.4f (Must be <= 1.0 for feasibility)', gHard);
fprintf('\nSoft Goal Value      (fSoft): %.4f\n', fSoft);

% FIX-B: getBlockValue returns the TUNED NUMERIC DATA for a tunableSurface
% block, not a full tunableSurface object -- calling evalSurf/viewSurf on
% that raw data directly fails ("Undefined function 'evalSurf' for input
% arguments of type ...", same symptom as in the minimal demo). Fix is to
% feed the tuned data back into the ORIGINAL template object (which still
% carries domain + shapeFcn) via setData, and use THAT for evalSurf/viewSurf.
Kp_tuned_data = getBlockValue(CL_MIMO_Tuned, 'Kp_Full');
Ki_tuned_data = getBlockValue(CL_MIMO_Tuned, 'Ki_Full');
Kd_tuned_data = getBlockValue(CL_MIMO_Tuned, 'Kd_Full');

Kp_LPV_Tuned = setData(Kp_Surface, Kp_tuned_data);
Ki_LPV_Tuned = setData(Ki_Surface, Ki_tuned_data);
Kd_LPV_Tuned = setData(Kd_Surface, Kd_tuned_data);

% Evaluate gain matrices at each specific stage for flight software implementation
K_Stages = cell(n_samples, 1);
for s = 1:n_samples
    Kp_s = evalSurf(Kp_LPV_Tuned, struct('S', s));
    Ki_s = evalSurf(Ki_LPV_Tuned, struct('S', s));
    Kd_s = evalSurf(Kd_LPV_Tuned, struct('S', s));
    K_Stages{s} = [Kp_s, Ki_s, Kd_s];
end

save('K_10DOF_LPV_HighlyCoupled.mat', 'Kp_LPV_Tuned', 'Ki_LPV_Tuned', 'Kd_LPV_Tuned', 'K_Stages', 'CL_MIMO_Tuned');
fprintf('Successfully saved LPV Controller Surfaces and Discrete Lookups to "K_10DOF_LPV_HighlyCoupled.mat".\n');

% --- Visual Verification across Stage Grid ---
figure('Name', 'LPV Gain Evolution vs Assembly Stage');
viewSurf(Kp_LPV_Tuned(1,1)); title('Base Axis 1 Kp Evolution across Assembly Stages');

figure('Name', 'MIMO Sensitivity & Cross-Talk (All Stages)');
viewGoal(Req_Sens_arm, CL_MIMO_Tuned);

figure('Name', 'Environmental Disturbance Rejection (All Stages)');
subplot(2,1,1); viewGoal(Req_Dist_Arm, CL_MIMO_Tuned); title('Arm Tip Disturbance across Stages');
subplot(2,1,2); viewGoal(Req_Dist_AOCS, CL_MIMO_Tuned); title('Base Disturbance across Stages');

%% FUNCTIONS
function tile_states = buildTileStates(n_tiles)
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