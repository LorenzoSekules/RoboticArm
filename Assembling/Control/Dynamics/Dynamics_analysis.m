%% PHASE 0: Define Signal Names matching Simulink Root Ports
gen_names = @(prefix, n) arrayfun(@(x) sprintf('%s(%d)', prefix, x), 1:n, 'UniformOutput', false);

% --- CONTROL INPUTS (u) - Must be last for LFT ---
W_ext  = gen_names('W_ext', 3);

T_RA = gen_names('T_RA',7);

all_inputs = [W_ext, T_RA];

% --- MEASUREMENT OUTPUTS (v) - Consumed by LFT ---
omega_dot = gen_names('omega_dot', 3);
Joint_acc = gen_names('Joint_acc', 7);

all_outputs = [omega_dot, Joint_acc];


%% PART 1: Extract Uncertain Plant Array
sdtModel = 'SDT_Dynamics';
open('SDT_Dynamics.slx');
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
    
    sys_open_loop = ulinearize('SDT_Dynamics');
    sys_open_loop = minreal(sys_open_loop);

    % Force the I/O names to match string arrays based on Simulink port order
    sys_open_loop.u  = all_inputs;
    sys_open_loop.y = all_outputs;
    
    G_SDT_samples{i} = sys_open_loop;
end

% Stack into a Multi-Model Uncertain State-Space Array
G_array = stack(1, G_SDT_samples{:});

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
