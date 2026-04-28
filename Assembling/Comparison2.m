% Comparison2: frequency-domain comparison between SDT and Simscape models
% for multiple full-DOF robotic-arm configurations sampled from q_traj.

%clear; clc;

load('best_trajectory2.mat')
Satellite_Data

sdtModel = 'Robot4_SDT';
simModel = 'Robot3_SIMSCAPE';

open(sdtModel);
open(simModel);

Name_SIM = simModel;
openMultibodyExplorer(Name_SIM);

% Build a sequential tile-placement schedule (one tile on the arm at a time).
tile_states = buildTileStates(7);

% Sample only configurations that are physically realized along q_traj.
n_samples = numel(tile_states);
traj_len = size(q_traj, 2);
traj_indices = round(linspace(1, traj_len, n_samples));
q_samples = q_traj(:, traj_indices);

G_SDT_samples = cell(n_samples, 1);
G_SIMSCAPE_samples = cell(n_samples, 1);

for i = 1:n_samples
    q_i = q_samples(:, i);

    % Update tile placement variables in the workspace for this sample.
    placements = tile_states(i).placements;
    Tile1_Placement = placements(1);
    Tile2_Placement = placements(2);
    Tile3_Placement = placements(3);
    Tile4_Placement = placements(4);
    Tile5_Placement = placements(5);
    Tile6_Placement = placements(6);
    Tile7_Placement = placements(7);

    % SDT: re-linearize for the current tile placement state.
    Gu = ulinearize(sdtModel);
    Gum = minreal(Gu);

    % SDT: substitute all 7 DOF using tan(theta/4) parameters.
    G_sdt_unc = usubs(Gum, ...
        'tan_Q_1_div4', tan(q_i(1) / 4), ...
        'tan_Q_2_div4', tan(q_i(2) / 4), ...
        'tan_Q_3_div4', tan(q_i(3) / 4), ...
        'tan_Q_4_div4', tan(q_i(4) / 4), ...
        'tan_Q_5_div4', tan(q_i(5) / 4), ...
        'tan_Q_6_div4', tan(q_i(6) / 4), ...
        'tan_Q_7_div4', tan(q_i(7) / 4));
    G_SDT_samples{i} = G_sdt_unc.NominalValue;

    % Simscape: set joint configuration directly in the workspace.
    robot.config_SIM = q_i.';
    set_param(Name_SIM, 'SimulationCommand', 'update');
    drawnow;
    G_SIMSCAPE_samples{i} = linearize(Name_SIM); % Simscape Linearization
    
end

%% Plot the Differences
figure();
hold on;

for i = 1:n_samples
    sigma(G_SDT_samples{i} - G_SIMSCAPE_samples{i});
end

grid on;
title(sprintf('Frequency-Domain Error (SDT vs Simscape) across %d Trajectory Samples', n_samples));
legend('Error (SDT - Simscape)', 'AutoUpdate', 'off');
hold off;


function openMultibodyExplorer(modelName)
% openMultibodyExplorer opens the Simscape Multibody Explorer if available.

if exist('smexplr', 'file') == 2
    smexplr(modelName);
    return;
end

if exist('smexplore', 'file') == 2
    smexplore(modelName);
    return;
end

warning('Simscape Multibody Explorer function not found. Skipping explorer open.');
end


function tile_states = buildTileStates(n_tiles)
% buildTileStates returns a sequential placement schedule with a single tile on the arm.
% State 1 = start, State 2 = on end-effector, State 3 = placed on antenna.

states = struct('name', {}, 'placements', {});
idx = 1;

states(idx).name = 'start';
states(idx).placements = ones(1, n_tiles);
idx = idx + 1;

for k = 1:n_tiles
    placements = ones(1, n_tiles);
    if k > 1
        placements(1:k-1) = 3;
    end
    placements(k) = 2;
    states(idx).name = sprintf('tile%d_grab', k);
    states(idx).placements = placements;
    idx = idx + 1;

    placements(k) = 3;
    states(idx).name = sprintf('tile%d_placed', k);
    states(idx).placements = placements;
    idx = idx + 1;
end

tile_states = states;
end