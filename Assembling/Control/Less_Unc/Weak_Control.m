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

%% Trajectory

% 1. Carica i dati della traiettoria originale
load('best_trajectory_def.mat');

traj_options = struct();
traj_options.totalTime = 12.0;
traj_options.numSamples = 241;

q_start_zero = zeros(7, 1);
q_first_wp   = q_traj(:, 1);

[t_init, q_init, qd_init, qdd_init] = trajectoryGeneration(q_start_zero, q_first_wp, traj_options);

% 3. Accoda il nuovo segmento (rimuovendo l'ultimo campione per evitare duplicati)
% In questo modo q_traj, qd_traj e qdd_traj sono perfettamente continui
q_traj   = [q_init(:, 1:end-1), q_traj];
qd_traj  = [qd_init(:, 1:end-1), qd_traj];
qdd_traj = [qdd_init(:, 1:end-1), qdd_traj];

% Aggiorniamo anche il vettore tempi traslando quello vecchio
t_vec = [t_init(1:end-1), t_vec + t_init(end)];

numSamples = 241; %from main
segment_length = numSamples - 1;

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
    if i == 1
        seg_start = 0;
        seg_end   = 2;
    elseif i == 15
        seg_start = 3 * i - 4;
        seg_end   = 42;
    else
        % From configuration 2 onward, the local windows advance by 3 samples:
        % i=2 -> 3,4,5 ; i=3 -> 6,7,8 ; ... ; i=14 -> 39,40,41
        seg_start = 3 * i - 4;
        seg_end   = 3 * i - 1;
    end

    % Map the segment indices to the actual q_traj samples.
    idx_start = seg_start * segment_length + 1;
    idx_end   = seg_end * segment_length + 1;

    % Extract the exact trajectory subset for this local configuration.
    q_traj_conf = q_traj(:, idx_start:idx_end);

    % Use the center of the local segment as the nominal linearization point.
    idx_nominale = round((idx_start + idx_end) / 2);
    q_i = q_traj(:, idx_nominale);

    Data_sat_Nominal;


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

    % --- FIX: Rinomina le incertezze cinematiche locali per evitare conflitti nello stack ---
    % Estraiamo i nomi di tutte le variabili incerte nel modello corrente
    if isa(sys_open_loop, 'uss')
        unc_names = fieldnames(sys_open_loop.Uncertainty);
        
        for v = 1:length(unc_names)
            var_name = unc_names{v};
            
            % Se l'incertezza riguarda i giunti (contiene 'Q_'), la rinominiamo
            if contains(var_name, 'Q_')
                old_u = sys_open_loop.Uncertainty.(var_name);
                
                % Creiamo un nome univoco per questo campione (es: tan_Q_1_div4_samp1)
                new_name = sprintf('%s_samp%d', var_name, i);
                
                % Ricreiamo l'ureal con gli stessi valori ma nome nuovo
                new_u = ureal(new_name, old_u.NominalValue, 'Range', old_u.Range);
                
                % Sostituiamo la vecchia variabile con la nuova nel modello
                sys_open_loop = usubs(sys_open_loop, var_name, new_u);
            end
        end
    end
    % ----------------------------------------------------------------------------------------

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

load("K_arm_tuned2.mat");

% --- BUILD GAINS ---
% logical(eye(7)) forces strictly decentralized control (only 7 tunable parameters each)
Kp_Arm = tunableGain('Kp_Arm', eye(7)); 
Kp_Arm.Gain.Free = logical(eye(7));
Kp_Arm.Gain.Value = K_arm_tuned.D(1:7,1:7); %eye(7);

Ki_Arm = tunableGain('Ki_Arm', eye(7)); 
Ki_Arm.Gain.Free = logical(eye(7));
Ki_Arm.Gain.Value = K_arm_tuned.D(1:7,8:14); %eye(7);

Kd_Arm = tunableGain('Kd_Arm', eye(7)); 
Kd_Arm.Gain.Free = logical(eye(7));
Kd_Arm.Gain.Value = K_arm_tuned.D(1:7,15:21); %eye(7);

% s=tf('s');
% 
% % Placing a pole up high to meka a proper controller
% for i =1:7
% tau = Kd_Arm(i,i)/10/Kp_Arm(i,i); % Polo a 20 rad/s (modifica in base al rumore dei tuoi encoder)
% Fd_arm(i,i)  = 1 / (tau * s + 1);
% end
% 
% Kd_Arm = Kd_Arm * Fd_arm;

% 2. Concatenate into a single 7x21 matrix [Kp, Ki, Kd]
K_Arm_Matrix = [Kp_Arm, Ki_Arm, Kd_Arm];
K_Arm_Matrix.InputName  = pid_arm_inputs;
K_Arm_Matrix.OutputName = u_arm;

% 3. Close the loop via LFT
CL_Arm = lft(G_arm_isolated, K_Arm_Matrix);

% Define systune options
opt = systuneOptions('MaxIter', 300, 'RandomStart', 2, 'Display', 'iter','UseParallel',true);

% ---------------------------------------------------------
% 1. TRACKING GOAL (Bandwidth of 1 rad/s)
wm = 5;
Req_Arm_Track = TuningGoal.Tracking(ref_arm, q_arm, wm);

% ---------------------------------------------------------
% 2. SENSITIVITY GOAL S(s) (Robust Stability)
% Limit transfer function from Ref to Error to a peak of 2.0
W_Sens = makeweight(0.01, wm, 2);
Req_Arm_Sens = TuningGoal.Gain(ref_arm, err_arm, W_Sens);

% ---------------------------------------------------------
% 3. DISTURBANCE REJECTION S_d(s)
% Coherent with bandwidth: tight at low freq (0.01), loose at high freq (10)
W_dist = makeweight(0.015/(5.15*13.5), 0.1*5e2, 10);
Req_Arm_Dist = TuningGoal.Gain(disturb_arm, q_arm, W_dist);

% ---------------------------------------------------------
% 4. CLASSIC CONTROLLER ROLL-OFF K(s)S(s)
% Replaces MaxLoopGain. Forces the controller to act as a low-pass filter.
%W_ControlEffort = makeweight(10, 5, 0.1);
Req_Control = TuningGoal.Gain(ref_arm, torque_arm, 1500/3.11);

%Req_poles = TuningGoal.Poles(10^-3,0.6,inf);

% figure()
% bodemag(W_dist)
% hold on
% bodemag(W_ControlEffort)
% legend('disturbances', 'commande','Fontsize', 20)
% grid on
% axis on

% =========================================================
% RUN SYSTUNE
% =========================================================

% Soft Goals: Try to track references and reject disturbances
Soft_Goals = [];

% Hard Goals: NEVER exceed Sensitivity of 2.0, NEVER excite fast dynamics
Hard_Goals = [Req_Arm_Sens,Req_Control,Req_Arm_Dist];

[CL_Arm_tuned, fSoft, gHard] = systune(CL_Arm, Soft_Goals, Hard_Goals, opt); 

% =========================================================
% EXTRACT TUNED CONTROLLER GAINS
% =========================================================
fprintf('\nExtracting Tuned PID Gains...\n');

% 1. Extract the numerical matrices from the tuned closed-loop model
Kp_opt = getBlockValue(CL_Arm_tuned, 'Kp_Arm');
Ki_opt = getBlockValue(CL_Arm_tuned, 'Ki_Arm');
Kd_opt = getBlockValue(CL_Arm_tuned, 'Kd_Arm');

% 2. Unify them into a single matrix. 
% We use the exact same [-Kp, Ki, -Kd] structure you used to build it earlier.
K_arm_tuned = [Kp_opt, Ki_opt, Kd_opt];

% 3. Display the final unified 7x21 matrix in the command window
disp('Final Tuned Controller Matrix [ -Kp | Ki | -Kd ]:');
disp(K_arm_tuned);

% Save K to the current folder
save('K_arm_tuned3.mat', 'K_arm_tuned');

%% VIEW GOAL ARM
figure()
viewGoal(Req_Control,CL_Arm_tuned)
title('S(s)*K(s)')

figure()
viewGoal(Req_Arm_Dist,CL_Arm_tuned)
title('D(s)')

figure()
viewGoal(Req_Arm_Sens,CL_Arm_tuned)
title('S(s)')

figure()
viewGoal(Req_Arm_Track,CL_Arm_tuned)
title('F(s)')

%% PART 3: Tune the Decentralized ADCS Controller (3-Axis)
fprintf('\n--- PHASE 2: Tuning the 3-Axis ADCS Controller ---\n');

% 1. Lock the tuned arm controller back into the full plant using LFT
G_array_with_arm = lft(G_array, K_arm_tuned); %if I have just computed K_arm_tuned
%G_array_with_arm = lft(G_array, K_Arm_Matrix); % if computing just AOCS control

% 2. Create Tunable Diagonal Gains for the base
Kp_AOCS = tunableGain('Kp_AOCS', eye(3));
Kp_AOCS.Gain.Free = logical(eye(3)); 
Kp_AOCS.Gain.Value = eye(3);

Ki_AOCS = tunableGain('Ki_AOCS', eye(3));
Ki_AOCS.Gain.Free = logical(eye(3));
Ki_AOCS.Gain.Value = eye(3);

Kd_AOCS = tunableGain('Kd_AOCS', eye(3));
Kd_AOCS.Gain.Free = logical(eye(3));
Kd_AOCS.Gain.Value = eye(3);

% s=tf('s');
% 
% % Placing a pole up high to meka a proper controller
% for i =1:3
% %tau = Kd_AOCS(i,i)/10/Kp_AOCS(i,i); % Polo a 2 rad/s
% tau = 0.5;
% Fd_aocs(i,i)  = 1 / (tau * s + 1);
% end
% 
% Kd_AOCS = Kd_AOCS * Fd_aocs;

% 3. Concatenate into a 3x9 matrix [Kp, Ki, Kd]
K_AOCS_Matrix = [Kp_AOCS, Ki_AOCS, Kd_AOCS];
K_AOCS_Matrix.InputName  = pid_aocs_inputs;
K_AOCS_Matrix.OutputName = u_aocs;

% 4. Close the final ADCS loop via LFT
CL_Full = lft(G_array_with_arm, K_AOCS_Matrix);

% --- ADCS TUNING GOALS ---
wt = 0.01;
%Req_AOCS_Track  = TuningGoal.Tracking(ref_aocs, q_aocs, wt);

W_Sens = makeweight(0.01, wt, 2);
Req_AOCS_Sens   = TuningGoal.Gain(ref_aocs, err_aocs, W_Sens);

Req_AOCS_Effort = TuningGoal.Gain(ref_aocs, torque_aocs, 0.82/deg2rad(3)); 

W_dist = makeweight(0.0047, [wm, deg2rad(3)/11], 1);
Req_AOCS_Dist   = TuningGoal.Gain(disturb_aocs, q_aocs, W_dist);

%ReqPointing = TuningGoal.Gain(disturb_aocs, err_aocs, deg2rad(3)/11);

% Soft Goals: Try to track references and reject disturbances
Soft_Goals = [];

% Hard Goals: NEVER exceed Sensitivity of 2.0, NEVER excite fast dynamics
Hard_Goals = [Req_AOCS_Sens,Req_AOCS_Effort,Req_AOCS_Dist];

opt = systuneOptions('MaxIter', 300, 'RandomStart', 1, 'Display', 'iter','UseParallel',true);

[CL_Full_tuned, fSoft, gHard] = systune(CL_Full, Soft_Goals, Hard_Goals, opt);

% =========================================================
% EXTRACT TUNED CONTROLLER GAINS
% =========================================================
fprintf('\nExtracting Tuned PID Gains...\n');

% 1. Extract the numerical matrices from the tuned closed-loop model
Kp_opt_aocs = getBlockValue(CL_Full_tuned, 'Kp_AOCS');
Ki_opt_aocs = getBlockValue(CL_Full_tuned, 'Ki_AOCS');
Kd_opt_aocs = getBlockValue(CL_Full_tuned, 'Kd_AOCS');

% 2. Unify them into a single matrix. 
% We use the exact same [Kp, Ki, Kd] structure you used to build it earlier.
K_aocs_tuned = [Kp_opt_aocs, Ki_opt_aocs, Kd_opt_aocs];

% 3. Display the final unified 7x21 matrix in the command window
disp('Final Tuned Controller Matrix [ Kp | Ki | Kd ]:');
disp(K_aocs_tuned);

% Save K to the current folder
save('K_aocs_tuned3.mat', 'K_aocs_tuned');

%% VIEWGOALS

figure()
viewGoal(Req_AOCS_Effort,CL_Full_tuned)
title('S(s)*K(s)')

figure()
viewGoal(Req_AOCS_Dist,CL_Full_tuned)
title('D(s)')

figure()
viewGoal(Req_AOCS_Sens,CL_Full_tuned)
title('S(s)')

figure()
viewGoal(Req_AOCS_Effort_on_Dist,CL_Full_tuned)


%% PART 4: View Final Gains
fprintf('\n===================================================\n');
fprintf('ROBUST TUNING COMPLETE\n');
fprintf('===================================================\n');

disp('--> SATELLITE BASE ADCS GAINS (Roll, Pitch, Yaw):');
showTunable(K_aocs_tuned)

disp('--> ROBOTIC ARM DECENTRALIZED GAINS (Joints 1 to 7):');
showTunable(K_arm_tuned)

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

