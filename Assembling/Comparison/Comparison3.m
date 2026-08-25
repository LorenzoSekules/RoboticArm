% Comparison2: frequency-domain comparison between SDT and Simscape models.
% The script samples full-DOF arm configurations from q_traj, applies a
% consistent tile-placement sequence, and compares linearized models using
% singular-value error metrics in dB.

clear; clc;

% --- Uncertainties ---
UNC_MAIN_BODY   = false;  % Incertezza su massa, inerzia e CoM del satellite
UNC_CONTAINER   = false;  % Incertezza sul container
UNC_BRICK       = false;  % Incertezza sui mattoncini
UNC_ROBOT_LINKS = false;  % Incertezza su masse/inerzie dei link del braccio
UNC_TILE        = false;  % Incertezza sulle tile
UNC_BOOM        = true;   % Incertezza sulla flessibilità della boom (Nastran)
UNC_JOINTS      = true;   % Incertezza sulla traiettoria angolare (q_traj)


% Load nominal trajectory and parameter definitions.

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
%Satellite_Data
%Data_sat_Nominal

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
Gum = cell(n_samples, 1);

numSamples = 241; %from main
segment_length = numSamples - 1;

for i = 1:n_samples
    clear q_traj_conf

    % Define the local trajectory segment associated with this sample.
    % The joint uncertainty range is then derived from the actual min/max of
    % that segment instead of assuming a constant span for the whole mission.
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
    q_i = q_traj(:, randi([idx_start, idx_end]));

    Data_sat_Nominal;

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
    Gum{i} = (Gu);

    % SDT: substitute all 7 DOF using tan(theta/4) parameters.
    G_sdt_unc = usubs(Gum{i}, ...
        'tan_Q_1_div4', tan(q_i(1) / 4), ...
        'tan_Q_2_div4', tan(q_i(2) / 4), ...
        'tan_Q_3_div4', tan(q_i(3) / 4), ...
        'tan_Q_4_div4', tan(q_i(4) / 4), ...
        'tan_Q_5_div4', tan(q_i(5) / 4), ...
        'tan_Q_6_div4', tan(q_i(6) / 4), ...
        'tan_Q_7_div4', tan(q_i(7) / 4));
    G_SDT_samples{i} = minreal(G_sdt_unc.NominalValue);

    % Simscape: set joint configuration directly in the workspace.
    robot.config_SIM = q_i.';
    set_param(Name_SIM, 'SimulationCommand', 'update');
    drawnow;
    G_SIMSCAPE_samples{i} = (linearize(Name_SIM)); % Simscape Linearization
    
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

%% --- Analisi del Worst-Case Scenario con Incertezze (Monte Carlo) ---

% 1. Trova l'indice della configurazione con l'errore di picco massimo
[~, idx_worst] = max(peak_max_sv_db);

if iscell(state_names)
    worst_name = state_names{idx_worst};
else
    worst_name = sprintf('Sample %d', idx_worst);
end

fprintf('\nWorst-Case Scenario identificato: %s (Sample ID: %d)\n', worst_name, idx_worst);

% 2. Estrai i modelli associati al worst-case
G_SDT_worst_nom = G_SDT_samples{idx_worst};
G_SIM_worst     = G_SIMSCAPE_samples{idx_worst};
G_SDT_worst_unc = Gum{idx_worst};

% Definisci il modello dell'errore
G_ERR = G_SDT_worst_nom - G_SIM_worst;

% 3. Genera N_mc campioni randomici (Monte Carlo) variando gli ureal della boom
N_mc = 30; 
if isa(G_SDT_worst_unc, 'uss')
    G_SDT_mc = usample(G_SDT_worst_unc, N_mc);
else
    G_SDT_mc = repmat(G_SDT_worst_nom, 1, 1, N_mc); 
end

% 4. Plot nativo usando la funzione "sigma" 
figure('Color', 'w', 'Name', ['Worst-Case: ', worst_name], 'NumberTitle', 'off');
hold on; % Fondamentale metterlo PRIMA dei plot sigma

% Plottiamo l'array di modelli incerti in ciano (così fa da sfondo)
%sigma(G_SDT_mc, 'c', w);

% Plottiamo i sistemi principali
sigma(G_SDT_worst_nom, 'b', w);  % SDT Nominale (Blu)
sigma(G_SIM_worst, 'g--', w);    % Simscape (Verde tratteggiato)
sigma(G_ERR, 'r', w);            % Errore (Rosso)

grid on;
title(sprintf('Sigma Plot Completo - Worst-Case Scenario (%s)', worst_name));

% % 5. Pulizia della legenda
% % (Poiché MATLAB inserirebbe una voce per ogni singolo sample incerto, 
% % creiamo delle linee "fittizie" invisibili solo per agganciarci la legenda ordinata)
% h1 = plot(nan, nan, 'c', 'LineWidth', 2);
% h2 = plot(nan, nan, 'b', 'LineWidth', 1.5);
% h3 = plot(nan, nan, 'g--', 'LineWidth', 1.5);
% h4 = plot(nan, nan, 'r', 'LineWidth', 1.5);
% 
% legend([h1, h2, h3, h4], ...
%     {'SDT Uncertain Samples (Boom var.)', 'G_{SDT} (Nominal)', 'G_{SIM} (Simscape)', 'Error |G_{SDT} - G_{SIM}|'}, ...
%     'Location', 'southwest');
%% Functions

function [t, q, qd, qdd] = trajectoryGeneration(q_start, q_goal, options)
% TRAJECTORYGENERATION Minimum-jerk quintic interpolation in joint space.
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