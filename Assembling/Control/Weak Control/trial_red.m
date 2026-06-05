% Pre-allocate cell arrays for speed
G_SDT_original = cell(n_samples, 1);
G_SDT_reduced  = cell(n_samples, 1);

% Define your target order for the physical plant
% (You can adjust this based on what the view(Rspec) showed you earlier)
reduced_order = 18; 

fprintf('Extracting and Reducing %d Physical Models...\n', n_samples);
open('SDT_Dynamics.slx')

for i = 1:n_samples
    % 1. Set your operating points for the current tile/sample
    q_i = q_samples(:, i);
    placements = tile_states(i).placements;
    Tile1_Placement = placements(1);
    Tile2_Placement = placements(2);
    Tile3_Placement = placements(3);
    Tile4_Placement = placements(4);
    Tile5_Placement = placements(5);
    Tile6_Placement = placements(6);
    Tile7_Placement = placements(7);
    
    % 2. Linearize ONLY the physical plant
    sys_dyn = ulinearize('SDT_Dynamics');
    sys_dyn = minreal(sys_dyn);
    
    % Save the original uncertain model
    G_SDT_original{i} = sys_dyn;
    
    % ==========================================================
    % 3. REDUCTION OF THE UNCERTAIN MODEL
    % ==========================================================
    % Separate the nominal model (M) and the uncertainty (Delta)
    [M, Delta] = lftdata(sys_dyn);
    
    % Reduce ONLY the nominal physical model using MatchDC
    Rspec = reducespec(M, "balanced");
    figure('Name', 'Hankel Singular Values');
    view(Rspec);
    M_red = getrom(Rspec, Order=reduced_order, Method="matchDC");
    
    % Re-attach uncertainty to create the reduced 'uss' model
    sys_dyn_red = lft(Delta, M_red);
    G_SDT_reduced{i} = sys_dyn_red;
    
    % ==========================================================
    % 4. GENERATE THE 15 VALIDATION FIGURES
    % ==========================================================
    figure('Name', sprintf('Plant %d: Original vs Reduced', i));
    
    % We use sigmaplot to compare MIMO systems (Torques to Accelerations)
    % Blue solid line = Original, Red dashed line = Reduced
    sigmaplot(sys_dyn, 'b', sys_dyn_red, 'g',sys_dyn-sys_dyn_red,'r');
    title(sprintf('Model %d Match: Torques to Accelerations', i));
    legend('Original (uss)', sprintf('Reduced (%d states)', reduced_order));
    grid on;
    
    % 'drawnow' forces MATLAB to render the figure immediately so it 
    % doesn't freeze your screen while computing the loop
    drawnow; 
end

% ==========================================================
% 5. CREATE THE FINAL TOTAL STACKS
% ==========================================================
fprintf('\nStacking models...\n');

% Stack the 15 reduced, uncertain models into a single array
G_array_reduced = stack(1, G_SDT_reduced{:});

% (Optional) Stack the original ones just in case you need them for comparison later
% G_array_original = stack(1, G_SDT_original{:});

fprintf('Success! Reduced stack created with %d states per model.\n', order(G_array_reduced(:,:,1)));

%%
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

