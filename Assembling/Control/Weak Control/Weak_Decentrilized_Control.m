%% =========================================================================
% MASTER SATELLITE & 7-DOF ARM CONTROL TUNING SCRIPT
% Architecture: Weakly Coupled (Sequential) + LFT + 21x1 Vectorized Output
% Features: Explicit Sensitivity (S) Function Tuning via Error Outports
%% =========================================================================

%% PHASE 0: Define Signal Names matching Simulink Root Ports
gen_names = @(prefix, n) arrayfun(@(x) sprintf('%s(%d)', prefix, x), 1:n, 'UniformOutput', false);

% --- EXOGENOUS INPUTS (w) ---
ref_aocs     = gen_names('Ref_AOCS', 3);
ref_arm      = gen_names('Ref_Arm', 7);
disturb_aocs = gen_names('Disturb_AOCS', 3);
disturb_arm  = gen_names('Disturb_Arm', 7);

% --- CONTROL INPUTS (u) - Must be last for LFT ---
u_aocs  = gen_names('u_AOCS', 3);
u_arm   = gen_names('u_Arm', 7);

all_inputs = [ref_aocs, ref_arm, disturb_aocs, disturb_arm, u_aocs, u_arm];

% --- EXOGENOUS OUTPUTS (z) - Exposed for Performance Evaluation ---
q_aocs           = gen_names('q_AOCS', 3);
q_arm            = gen_names('q_Arm', 7);
err_aocs         = gen_names('Err_AOCS', 3);  
err_arm          = gen_names('Err_Arm', 7);
torque_aocs      = gen_names('Torque_AOCS', 3);
torque_arm       = gen_names('Torque_Arm', 7);

% --- MEASUREMENT OUTPUTS (v) - Consumed by LFT ---
pid_aocs_inputs = gen_names('PID_AOCS_Out', 9);   % 9x1 vector (q, int_e, v)
pid_arm_inputs  = gen_names('PID_Arm_Out', 21);   % 21x1 vector (q, int_e, v)

all_outputs = [q_aocs, q_arm, err_aocs, err_arm, torque_aocs, torque_arm, ...
               pid_aocs_inputs, pid_arm_inputs];

%% PART 1: Extract Uncertain Plant Array
% sdtModel = 'SDT_Dynamics';
open('SDT_Control_Tuning.slx');
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
    
    sys_open_loop = ulinearize('SDT_Control_Tuning');
    sys_open_loop = minreal(sys_open_loop);

    % Force the I/O names to match string arrays based on Simulink port order
    sys_open_loop.u  = all_inputs;
    sys_open_loop.y = all_outputs;
    
    G_SDT_samples{i} = sys_open_loop;
end

% Stack into a Multi-Model Uncertain State-Space Array
G_array = stack(1, G_SDT_samples{:});

%% PART 2: Tune the Decentralized Arm Controller (7-DOF)
fprintf('\n--- PHASE 1: Tuning the 7-DOF Arm Controller ---\n');

% Isolate Arm channels for sequential tuning (Include the new Error Outport!)
arm_in_idx  = [ref_arm, disturb_arm, u_arm]; % 2 ex + 1 control
arm_out_idx = [q_arm, torque_arm, err_arm, pid_arm_inputs]; % 3ex + 1 control
G_arm_isolated = G_array(arm_out_idx, arm_in_idx);

% --- BUILD GAINS ---
% logical(eye(7)) forces strictly decentralized control (only 7 tunable parameters each)
Kp_Arm = tunableGain('Kp_Arm', eye(7)); 
Kp_Arm.Gain.Free = logical(eye(7)); 

Ki_Arm = tunableGain('Ki_Arm', eye(7)); 
Ki_Arm.Gain.Free = logical(eye(7)); 

Kd_Arm = tunableGain('Kd_Arm', eye(7)); 
Kd_Arm.Gain.Free = logical(eye(7)); 

% 2. Concatenate into a single 7x21 matrix [-Kp, Ki, -Kd]
K_Arm_Matrix = [-Kp_Arm, Ki_Arm, -Kd_Arm];
K_Arm_Matrix.InputName  = pid_arm_inputs;
K_Arm_Matrix.OutputName = u_arm;

% 3. Close the loop via LFT
CL_Arm = lft(G_arm_isolated, K_Arm_Matrix);

% Define systune options
opt = systuneOptions('MaxIter', 500, 'RandomStart', 2,'Display','iter');

% --- ARM TUNING GOALS ---
wm = 1;
Req_Arm_Track  = TuningGoal.Tracking(ref_arm, q_arm, wm);
%Req_Arm_Sens   = TuningGoal.Sensitivity(ref_arm, err_arm, 2); % EXPLICIT SENSITIVITY S(s)
Req_Arm_Effort = TuningGoal.Gain(ref_arm, torque_arm, 10); 
Req_Arm_Dist   = TuningGoal.Gain(disturb_arm, q_arm, 0.1);  

[CL_Arm_tuned, fSoft, gHard] = systune(CL_Arm, [Req_Arm_Effort, Req_Arm_Dist], Req_Arm_Track,opt)

%% PART 3: Tune the Decentralized ADCS Controller (3-Axis)
fprintf('\n--- PHASE 2: Tuning the 3-Axis ADCS Controller ---\n');

% 1. Lock the tuned arm controller back into the full plant using LFT
G_array_with_arm = lft(G_array, K_arm_tuned); 

% 2. Create Tunable Diagonal Gains for the base
Kp_AOCS = tunableGain('Kp_AOCS', eye(3));
Kp_AOCS.Gain.Free = logical(eye(3)); 

Ki_AOCS = tunableGain('Ki_AOCS', eye(3));
Ki_AOCS.Gain.Free = logical(eye(3)); 

Kd_AOCS = tunableGain('Kd_AOCS', eye(3));
Kd_AOCS.Gain.Free = logical(eye(3)); 

% 3. Concatenate into a 3x9 matrix [Kp, Ki, -Kd]
K_AOCS_Matrix = [-Kp_AOCS, Ki_AOCS, -Kd_AOCS];
K_AOCS_Matrix.InputName  = pid_aocs_inputs;
K_AOCS_Matrix.OutputName = u_aocs;

% 4. Close the final ADCS loop via LFT
CL_Full = lft(G_array_with_arm, K_AOCS_Matrix);

% --- ADCS TUNING GOALS ---
wt = 0.1;
Req_AOCS_Track  = TuningGoal.Tracking(ref_aocs, q_aocs, wt);
Req_AOCS_Sens   = TuningGoal.Sensitivity(ref_aocs, err_aocs, wt); % EXPLICIT SENSITIVITY S(s)
Req_AOCS_Effort = TuningGoal.Gain(ref_aocs, torque_aocs, 1.5); 
Req_AOCS_Dist   = TuningGoal.Gain(disturb_aocs, q_aocs, 0.05);  

[CL_Full_tuned, K_aocs_tuned] = systune(CL_Full, Req_AOCS_Effort, [Req_AOCS_Track, Req_AOCS_Sens, Req_AOCS_Dist]);

%% PART 4: View Final Gains
fprintf('\n===================================================\n');
fprintf('ROBUST TUNING COMPLETE\n');
fprintf('===================================================\n');

disp('--> SATELLITE BASE ADCS GAINS (Roll, Pitch, Yaw):');
showTunable(K_aocs_tuned);

disp('--> ROBOTIC ARM DECENTRALIZED GAINS (Joints 1 to 7):');
showTunable(K_arm_tuned);

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

