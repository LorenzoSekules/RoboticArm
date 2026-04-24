%clear all; close all; clc;

%% 1. Load Data and Initialize Parameters
% Load satellite, container, tile, and arm data
Satellite_Data; 
load('best_trajectory2.mat'); % Assuming this loads q_traj (7xN matrix)

Name_SIM = 'Robot3_SIMSCAPE';
Name_SDT = 'Robot3_SDT';

% Open models
open(Name_SIM);
open(Name_SDT);

%% 2. Define the Sequence Logic
% We have 14 waypoints -> 13 segments.
% Odd segments (1, 3, 5...): Arm is moving from Container to Antenna (Carrying Tile)
% Even segments (2, 4, 6...): Arm is moving from Antenna to Container (Empty)
n_segments = 13;
total_samples = size(q_traj, 2);
samples_per_seg = total_samples / n_segments;

% Pre-allocate cell arrays to store the linearized state-space models
G_SIMSCAPE_sampled = cell(n_segments, 1);
G_SDT_sampled = cell(n_segments, 1);

% Arrays to track the variables for our plot legends
segment_names = cell(n_segments, 1);

%% 3. Linearization Loop over the 13 Trajectory Segments
for i = 1:n_segments
    
    % --- A. Extract Arm Posture (q_test) ---
    % Grab the posture at the exact middle of the current segment
    mid_idx = round((i - 0.5) * samples_per_seg);
    q_test = q_traj(:, mid_idx);
    
    % --- B. Enforce Tile Placement Logic ---
    % 1 = Container, 2 = Arm, 3 = Antenna
    Tile_Placement = ones(1, 7); % Default all 7 tiles to the Container
    
    % Determine which tile we are currently handling (Segment 1/2 -> Tile 1, etc.)
    current_tile = ceil(i / 2); 
    is_carrying = (mod(i, 2) ~= 0); % True if odd segment
    
    for t = 1:7
        if t < current_tile
            % Previous tiles are already placed on the antenna
            Tile_Placement(t) = 3; 
        elseif t == current_tile
            % The current tile depends on if the arm is carrying it or just dropped it
            if is_carrying
                Tile_Placement(t) = 2; % On the end-effector
                action_str = ['Carrying Tile ', num2str(t)];
            else
                Tile_Placement(t) = 3; % Dropped on antenna, arm returning
                action_str = ['Returning Empty (Tile ', num2str(t), ' Placed)'];
            end
        else
            % Future tiles are still in the container
            Tile_Placement(t) = 1; 
        end
    end
    
    % Push Variables to Workspace for Simulink/Simscape to read
    Tile1_Placement = Tile_Placement(1);
    Tile2_Placement = Tile_Placement(2);
    Tile3_Placement = Tile_Placement(3);
    Tile4_Placement = Tile_Placement(4);
    Tile5_Placement = Tile_Placement(5);
    Tile6_Placement = Tile_Placement(6);
    Tile7_Placement = Tile_Placement(7);
    
    segment_names{i} = sprintf('Seg %d: %s', i, action_str);
    disp(['Processing ', segment_names{i}, '...']);
    
    % --- C. Update Models & Linearize ---
    % Force both models to update their Variant Subsystems and Kinematics
    set_param(Name_SIM, 'SimulationCommand', 'update');
    set_param(Name_SDT, 'SimulationCommand', 'update');
    
    % Linearize Simscape
    G_SIMSCAPE_sampled{i} = linearize(Name_SIM);
    
    % Linearize SDT (Must re-linearize because topology changes!)
    Gu_tmp = ulinearize(Name_SDT);
    G_SDT_sampled{i} = minreal(Gu_tmp.NominalValue); % Clean up ghost states
    
    % Clear warnings to keep console clean
    warn1 = warning('query','last');
    warning('off',warn1.identifier);
end

disp('Validation Loop Complete. Generating Plots...');

%% 4. Frequency Domain Analysis & Plotting
% NOTE: You may need to adjust the exact string names in the cell arrays 
% below to match the exact names of your Simulink Root Inports/Outports!

% Define Input Labels (13 Inputs)
Inputs_Wrench = {'W_ext(1)', 'W_ext(2)', 'W_ext(3)', 'W_ext(4)', 'W_ext(5)', 'W_ext(6)'};
Inputs_Arm    = {'Tau_arm(1)', 'Tau_arm(2)', 'Tau_arm(3)', 'Tau_arm(4)', 'Tau_arm(5)', 'Tau_arm(6)', 'Tau_arm(7)'};

% Define Output Labels (13 Outputs)
Outputs_Bus   = {'Xddot_MB(1)', 'Xddot_MB(2)', 'Xddot_MB(3)', 'Xddot_MB(4)', 'Xddot_MB(5)', 'Xddot_MB(6)'};
Outputs_Arm   = {'qddot(1)', 'qddot(2)', 'qddot(3)', 'qddot(4)', 'qddot(5)', 'qddot(6)', 'qddot(7)'};

% A. Rigid Body Dynamics: External Wrench to Bus Accelerations
SDTlib_Simscape_SingularValues_plots_v2(G_SDT_sampled, G_SIMSCAPE_sampled,...
    Outputs_Bus, Inputs_Wrench,...
    '\ddot{X}_B(1 \rightarrow 6)', 'W_{ext}(1 \rightarrow 6)',...
    [-4, 4], 1000, 'Segment', 1:n_segments);

% B. Manipulator Dynamics: Arm Torques to Arm Accelerations
SDTlib_Simscape_SingularValues_plots_v2(G_SDT_sampled, G_SIMSCAPE_sampled,...
    Outputs_Arm, Inputs_Arm,...
    '\ddot{q}(1 \rightarrow 7)', '\tau_{arm}(1 \rightarrow 7)',...
    [-4, 4], 1000, 'Segment', 1:n_segments);

% C. Coupling (Disturbance): Arm Torques to Base Accelerations
SDTlib_Simscape_SingularValues_plots_v2(G_SDT_sampled, G_SIMSCAPE_sampled,...
    Outputs_Bus, Inputs_Arm,...
    '\ddot{X}_B(1 \rightarrow 6)', '\tau_{arm}(1 \rightarrow 7)',...
    [-4, 4], 1000, 'Segment', 1:n_segments);