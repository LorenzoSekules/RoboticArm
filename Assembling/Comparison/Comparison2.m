% Comparison2: frequency-domain comparison between SDT and Simscape models.
% The script samples full-DOF arm configurations from q_traj, applies a
% consistent tile-placement sequence, and compares linearized models using
% singular-value error metrics in dB.

%clear; clc;

% Load nominal trajectory and parameter definitions.
load('best_trajectory_def.mat')
Satellite_Data

sdtModel = 'Robot4_SDT';
simModel = 'Robot3_SIMSCAPE';

% Open the models once so the subsequent linearizations run faster.
open(sdtModel);
open(simModel);

Name_SIM = simModel;
% Keep the Multibody Explorer in sync with each sampled configuration.
openMultibodyExplorer(Name_SIM);

% Build a sequential tile-placement schedule (one tile on the arm at a time).
tile_states = buildTileStates(7);
state_names = {tile_states.name};

% Sample only configurations that are physically realized along q_traj.
n_samples = numel(tile_states);
traj_len = size(q_traj, 2);
traj_indices = round(linspace(1, traj_len, n_samples));
q_samples = q_traj(:, traj_indices);

% Store the linearized models for SDT and Simscape at each sample.
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

%% Post-processing and plots (dB)
[sv_ref, w] = sigma(G_SDT_samples{1} - G_SIMSCAPE_samples{1});
sv_ref = squeeze(sv_ref);
max_sv = zeros(n_samples, numel(w));

for i = 1:n_samples
    sv_i = sigma(G_SDT_samples{i} - G_SIMSCAPE_samples{i}, w);
    sv_i = squeeze(sv_i);
    max_sv(i, :) = max(sv_i, [], 1);
end

% Convert to dB and build envelope statistics for visualization.
max_sv_db = 20 * log10(max_sv + eps);
min_sv_db = min(max_sv_db, [], 1);
med_sv_db = median(max_sv_db, 1);
max_sv_env_db = max(max_sv_db, [], 1);
peak_max_sv_db = max(max_sv_db, [], 2);

figure('Color', 'w', 'Name', 'SDT vs Simscape Error Envelope', 'NumberTitle', 'off');
ax1 = axes();
hold(ax1, 'on');

for i = 1:n_samples
    plot(ax1, w, max_sv_db(i, :), 'Color', [0.78 0.78 0.78], 'LineWidth', 0.6, 'HandleVisibility', 'off');
end

% Dummy line so the sample family appears once in the legend.
plot(ax1, nan, nan, 'Color', [0.78 0.78 0.78], 'LineWidth', 0.6, 'DisplayName', 'Samples');

x_fill = [w(:); flipud(w(:))];
y_fill = [min_sv_db(:); flipud(max_sv_env_db(:))];
fill(ax1, x_fill, y_fill, [0.85 0.9 1], 'EdgeColor', 'none', 'FaceAlpha', 0.35, 'DisplayName', 'Envelope');
plot(ax1, w, med_sv_db, 'k', 'LineWidth', 1.8, 'DisplayName', 'Median');
plot(ax1, w, min_sv_db, '--', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.0, 'DisplayName', 'Min');
plot(ax1, w, max_sv_env_db, '--', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.0, 'DisplayName', 'Max');

set(ax1, 'XScale', 'log');
grid(ax1, 'on');
xlabel(ax1, 'Frequency [rad/s]');
ylabel(ax1, 'Max singular value |G_{SDT} - G_{SIM}| [dB]');
title(ax1, sprintf('Frequency-Domain Error Envelope across %d Samples', n_samples));
legend(ax1, 'Location', 'best');
hold(ax1, 'off');

figure('Color', 'w', 'Name', 'Peak Error per Sample', 'NumberTitle', 'off');
bar(peak_max_sv_db, 'FaceColor', [0.35 0.55 0.85]);
grid on;
title('Peak Max Singular Value per Sample');

% Removed xlabel('Sample index'); to avoid redundancy with the tick labels
ylabel('||G_{SDT} - G_{SIM}||_{\infty} [dB]', 'Interpreter', 'tex');

set(gca, 'XTick', 1:n_samples, 'XTickLabel', state_names, 'XTickLabelRotation', 30);


%% Functions

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

% function tile_states = buildTileStates(n_tiles)
% % buildTileStates returns a sequential placement schedule with a single tile on the arm.
% % State 1 = start, State 2 = on end-effector, State 3 = placed on antenna.
% 
% states = struct('name', {}, 'placements', {});
% idx = 1;
% 
% states(idx).name = 'start';
% states(idx).placements = ones(1, n_tiles);
% idx = idx + 1;
% 
% for k = 1:n_tiles
%     placements = ones(1, n_tiles);
%     if k > 1
%         placements(1:k-1) = 3;
%     end
%     placements(k) = 2;
%     states(idx).name = sprintf('tile%d_grab', k);
%     states(idx).placements = placements;
%     idx = idx + 1;
% 
%     placements(k) = 3;
%     states(idx).name = sprintf('tile%d_placed', k);
%     states(idx).placements = placements;
%     idx = idx + 1;
% end
% 
% tile_states = states;
% end