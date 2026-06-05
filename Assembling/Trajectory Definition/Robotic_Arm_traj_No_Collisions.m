function [t_vec, q_traj, qd_traj, qdd_traj] = Robotic_Arm_traj_No_Collisions(p_waypoints, R_waypoints)
    % ROBOTIC_ARM_TRAJ_NO_COLLISIONS
    % Planner Avanzato ad altissima precisione. Implementa l'Ancoraggio Cartesiano 
    % a 4 stadi per estrarre la tile da spazi angusti senza collisioni ad arco.
    
    disp('--- Avvio Planner Ottimizzato ad Energia Minima ---');

    % 1. Ricostruzione Timeline Payload
    n_expanded = size(p_waypoints, 2);
    n_wp = n_expanded / 3;
    expanded_poses = cell(1, n_expanded);
    payload_status = zeros(1, n_expanded);
    
    for k = 1:n_wp
        idx_pre  = 3 * (k - 1) + 1; idx_mid  = idx_pre + 1; idx_post = idx_pre + 2;
        expanded_poses{idx_pre}  = trvec2tform(p_waypoints(:, idx_pre)') * rotm2tform(R_waypoints(:, :, idx_pre));
        expanded_poses{idx_mid}  = trvec2tform(p_waypoints(:, idx_mid)') * rotm2tform(R_waypoints(:, :, idx_mid));
        expanded_poses{idx_post} = trvec2tform(p_waypoints(:, idx_post)') * rotm2tform(R_waypoints(:, :, idx_post));
        
        if mod(k, 2) == 1 % Pick
            payload_status(idx_pre) = 0; payload_status(idx_mid) = 1; payload_status(idx_post) = 1; 
        else % Drop
            payload_status(idx_pre) = 1; payload_status(idx_mid) = 0; payload_status(idx_post) = 0; 
        end
    end

    % 2. Ambiente Fisico
    env = {};
    bodyCol = collisionBox(2.2, 3, 5); bodyCol.Pose = trvec2tform([-1.1, 0, -0.5]); env{end+1} = bodyCol;
    
    Cx = 1.5; Cy = sqrt(3) + 0.5; Cz = sqrt(3) + 0.5; t = 0.05; 
    cont_center = [-0.75, 0, 2.0 + Cz/2]; 
    topCol = collisionBox(Cx, Cy, t); topCol.Pose = trvec2tform(cont_center + [0, 0, Cz/2 - t/2]); env{end+1} = topCol;
    botCol = collisionBox(Cx, Cy, t); botCol.Pose = trvec2tform(cont_center + [0, 0, -Cz/2 + t/2]); env{end+1} = botCol;
    backCol = collisionBox(t, Cy, Cz); backCol.Pose = trvec2tform(cont_center + [-Cx/2 + t/2, 0, 0]); env{end+1} = backCol;
    leftCol = collisionBox(Cx, t, Cz); leftCol.Pose = trvec2tform(cont_center + [0, -Cy/2 + t/2, 0]); env{end+1} = leftCol;
    rightCol = collisionBox(Cx, t, Cz); rightCol.Pose = trvec2tform(cont_center + [0, Cy/2 - t/2, 0]); env{end+1} = rightCol;
    
    brick1 = collisionBox(0.2, 0.2, 0.4); brick1.Pose = trvec2tform([-0.9, 0, -3.21]); env{end+1} = brick1;
    brick2 = collisionBox(0.2, 0.2, 0.4); brick2.Pose = trvec2tform([3.8, 0, -3.21]); env{end+1} = brick2;
    boomCol = collisionCylinder(0.1, 4.7); boomCol.Pose = trvec2tform([1.45, 0, -3.41]) * axang2tform([0 1 0, pi/2]); env{end+1} = boomCol;

    % 3. Modelli Robot
    robotEmpty = buildArm(false);
    robotLoaded = buildArm(true);

    % --- FASE 1: OTTIMIZZAZIONE GLOBALE DEI WAYPOINT ---
    disp('Calcolo Opzioni Cinematica Inversa (Dynamic Programming)...');
    gikEmpty = generalizedInverseKinematics('RigidBodyTree', robotEmpty, 'ConstraintInputs', {'pose'});
    gikLoaded = generalizedInverseKinematics('RigidBodyTree', robotLoaded, 'ConstraintInputs', {'pose'});
    poseConst = constraintPoseTarget('Link7'); 
    
    num_ik_opts = 8; 
    Q_options = cell(1, n_expanded);
    q_guess = robotEmpty.homeConfiguration;
    
    for i = 1:n_expanded
        poseConst.TargetTransform = expanded_poses{i};
        valid_qs = [];
        for attempt = 1:200
            if payload_status(i) == 1
                [q_sol, ~] = gikLoaded(q_guess, poseConst);
                isColl = checkCollision(robotLoaded, q_sol, env, "IgnoreSelfCollision", "on");
            else
                [q_sol, ~] = gikEmpty(q_guess, poseConst);
                isColl = checkCollision(robotEmpty, q_sol, env, "IgnoreSelfCollision", "on");
            end
            if ~isColl
                valid_qs = [valid_qs, q_sol];
                if size(valid_qs, 2) >= num_ik_opts, break; end
            end
            q_guess = (rand(7,1) - 0.5) * 2 * pi; 
        end
        if isempty(valid_qs), error('IK Fallita in modo critico al waypoint %d.', i); end
        Q_options{i} = valid_qs;
    end
    
    cost = cell(1, n_expanded); parent = cell(1, n_expanded);
    cost{1} = zeros(1, size(Q_options{1}, 2));
    for i = 2:n_expanded
        n_prev = size(Q_options{i-1}, 2); n_curr = size(Q_options{i}, 2);
        cost{i} = zeros(1, n_curr); parent{i} = zeros(1, n_curr);
        for c = 1:n_curr
            min_c = inf; best_p = 1;
            for p = 1:n_prev
                energy = norm(Q_options{i}(:,c) - Q_options{i-1}(:,p))^2; 
                tot = cost{i-1}(p) + energy;
                if tot < min_c, min_c = tot; best_p = p; end
            end
            cost{i}(c) = min_c; parent{i}(c) = best_p;
        end
    end
    
    q_waypoints = zeros(7, n_expanded);
    [~, curr_p] = min(cost{end}); q_waypoints(:, end) = Q_options{end}(:, curr_p);
    for i = n_expanded:-1:2
        curr_p = parent{i}(curr_p); q_waypoints(:, i-1) = Q_options{i-1}(:, curr_p);
    end

    % --- FASE 2: OTTIMIZZAZIONE LOCALE E CONTROLLO TILE ---
    disp('Generazione Traiettorie Locali (Ancoraggio Cartesiano Attivo)...');
    traj_opts = struct('totalTime', 12.0, 'numSamples', 241);
    half_opts = struct('totalTime', 6.0, 'numSamples', 121); 
    
    t_vec = []; q_traj = []; qd_traj = []; qdd_traj = []; is_loaded_traj = []; t_offset = 0;
    
    for i_seg = 1:(n_expanded - 1)
        q_start = q_waypoints(:, i_seg); q_goal = q_waypoints(:, i_seg + 1);
        isLoadedSegment = payload_status(i_seg) == 1;
        activeRobot = robotEmpty; if isLoadedSegment, activeRobot = robotLoaded; end
        
        if i_seg > 1 && payload_status(i_seg) == 0 && payload_status(i_seg - 1) == 1
            placedTile = collisionCylinder(1.0, 0.05);
            T_drop = getTransform(robotEmpty, q_start, 'Link7');
            placedTile.Pose = T_drop * trvec2tform([0, 0, 0.025]); 
            env{end+1} = placedTile; 
        end
        
        num_candidates = 30;
        cand_q = cell(1, num_candidates); cand_qd = cell(1, num_candidates);
        cand_qdd = cell(1, num_candidates); cand_E = zeros(1, num_candidates);
        
        if mod(i_seg, 3) ~= 0 
            % --- FASE DI APPROCCIO/RITIRO (Segmento 20, 4, ecc.) ---
            % L'ancoraggio cartesiano spezza l'arco in 4 linee rette infinitesimali.
            T_s = expanded_poses{i_seg}; T_g = expanded_poses{i_seg+1};
            q_vias_cart = zeros(7, 3);
            guess_ik = q_start;
            
            for step = 1:3
                alpha = step * 0.3;
                T_via = T_s; T_via(1:3,4) = T_s(1:3,4) * (1-alpha) + T_g(1:3,4) * alpha;
                poseConst.TargetTransform = T_via;
                if isLoadedSegment, [q_v, ~] = gikLoaded(guess_ik, poseConst);
                else, [q_v, ~] = gikEmpty(guess_ik, poseConst); end
                q_vias_cart(:, step) = q_v;
                guess_ik = q_v; % Usa il punto precedente per mantenere il gomito fermo!
            end
            
            quarter_opts = struct('totalTime', 3.0, 'numSamples', 61);
            [~, q1, qd1, qdd1] = trajectoryGeneration(q_start, q_vias_cart(:,1), quarter_opts);
            [~, q2, qd2, qdd2] = trajectoryGeneration(q_vias_cart(:,1), q_vias_cart(:,2), quarter_opts);
            [~, q3, qd3, qdd3] = trajectoryGeneration(q_vias_cart(:,2), q_vias_cart(:,3), quarter_opts);
            [~, q4, qd4, qdd4] = trajectoryGeneration(q_vias_cart(:,3), q_goal, quarter_opts);
            
            q_seg_direct = [q1, q2(:,2:end), q3(:,2:end), q4(:,2:end)];
            qd_seg_direct = [qd1, qd2(:,2:end), qd3(:,2:end), qd4(:,2:end)];
            qdd_seg_direct = [qdd1, qdd2(:,2:end), qdd3(:,2:end), qdd4(:,2:end)];
            
            for v = 1:num_candidates
                if v == 1
                    cand_q{v} = q_seg_direct; 
                else
                    % Aggiunge micro-rumore bianco nel caso la linea perfetta tocchi un angolo singolare
                    cand_q{v} = q_seg_direct + (randn(size(q_seg_direct)) * 0.002);
                end
                cand_qd{v} = qd_seg_direct; cand_qdd{v} = qdd_seg_direct;
                cand_E(v) = sum(sum(qd_seg_direct.^2)) + (v * 0.1); 
            end
            
        else 
            % --- FASE DI TRASFERIMENTO ---
            % Ricerca globale per aggirare l'antenna e i piedistalli
            ref = zeros(7,1); spread = pi; 
            for v = 1:num_candidates
                if v == 1
                    [~, q, qd, qdd] = trajectoryGeneration(q_start, q_goal, traj_opts);
                else
                    q_via = ref + (rand(7,1)-0.5) * spread;
                    [~, q1, qd1, qdd1] = trajectoryGeneration(q_start, q_via, half_opts);
                    [~, q2, qd2, qdd2] = trajectoryGeneration(q_via, q_goal, half_opts);
                    q = [q1, q2(:, 2:end)]; qd = [qd1, qd2(:, 2:end)]; qdd = [qdd1, qdd2(:, 2:end)];
                end
                cand_q{v} = q; cand_qd{v} = qd; cand_qdd{v} = qdd;
                cand_E(v) = sum(sum(qd.^2));
            end
        end
        
        [~, sorted_idx] = sort(cand_E);
        isSafe = false;
        
        for idx = 1:num_candidates
            c_idx = sorted_idx(idx); q_test = cand_q{c_idx};
            collisionDetected = false;
            
            for f = 1:size(q_test, 2)
                if checkCollision(activeRobot, q_test(:, f), env, "IgnoreSelfCollision", "off")
                    collisionDetected = true; break;
                end
                
                % BARRIERA RIGOROSA Z = -2.95m
                if isLoadedSegment
                    T_ee = getTransform(activeRobot, q_test(:, f), 'Link7');
                    lowest_z_tile = T_ee(3, 4) - 1.0 * norm(T_ee(3, 1:2));
                    
                    if lowest_z_tile < -2.952
                        collisionDetected = true; break;
                    end
                end
            end
            
            if ~collisionDetected
                q_seg = q_test; qd_seg = cand_qd{c_idx}; qdd_seg = cand_qdd{c_idx};
                isSafe = true;
                fprintf('  Segmento %d: Ottimizzato al candidato %d/%d (Energia Cinetica: %.1f)\n', i_seg, idx, num_candidates, cand_E(c_idx));
                break;
            end
        end
        
        if ~isSafe, error('FATAL: Nessuno dei %d candidati ha superato i controlli al segmento %d.', num_candidates, i_seg); end
        
        t_seg = linspace(0, 12, 241);
        if i_seg > 1
            t_seg = t_seg(2:end); q_seg = q_seg(:, 2:end); qd_seg = qd_seg(:, 2:end); qdd_seg = qdd_seg(:, 2:end);
        end
        
        status_array = repmat(payload_status(i_seg), 1, size(q_seg, 2));
        t_vec = [t_vec, t_seg + t_offset]; q_traj = [q_traj, q_seg]; %#ok<*AGROW>
        qd_traj = [qd_traj, qd_seg]; qdd_traj = [qdd_traj, qdd_seg];
        is_loaded_traj = [is_loaded_traj, status_array]; t_offset = t_vec(end);
    end
    disp('--- Ottimizzazione e Validazione Completata con Successo! ---');
    
    % --- FASE 3: ANIMAZIONE ---
    figure('Name', 'Robotic Arm Execution', 'Position', [100, 100, 800, 600]);
    ax = show(robotEmpty, q_traj(:, 1), 'PreservePlot', false, 'Frames', 'off'); hold on;
    for j = 1:length(env), show(env{j}, 'Parent', ax); end
    
    axis equal; view(135, 30); grid on; title('Esecuzione Ottimizzata a Minima Energia');
    xlim([-3, 5]); ylim([-4, 4]); zlim([-5, 5]);
    
    [X_tile, Y_tile, Z_tile] = cylinder(1.0, 30); Z_tile = Z_tile * 0.05 - 0.025; 
    tileTransform = hgtransform('Parent', ax);
    tileSurf = surf(X_tile, Y_tile, Z_tile, 'Parent', tileTransform, 'FaceColor', [0.9290 0.6940 0.1250], 'EdgeColor', 'none');
    set(tileSurf, 'Visible', 'off'); 
    
    numFrames = size(q_traj, 2); targetFPS = 30; playbackSpeedSeconds = 40; 
    frame_skip = max(1, floor(numFrames / (playbackSpeedSeconds * targetFPS)));
    
    for i = 1:frame_skip:numFrames
        show(robotEmpty, q_traj(:, i), 'FastUpdate', true, 'PreservePlot', false);
        if is_loaded_traj(i) == 1
            T_ee = getTransform(robotEmpty, q_traj(:, i), 'Link7');
            set(tileTransform, 'Matrix', T_ee * trvec2tform([0, 0, 0.025]));
            set(tileSurf, 'Visible', 'on');
        else
            set(tileSurf, 'Visible', 'off');
        end
        drawnow; 
    end
end

%% --- SUBFUNCTIONS ---
function robot = buildArm(addTile)
    robot = rigidBodyTree('DataFormat', 'column');
    dh_alpha = [-pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, 0]; dh_a = zeros(1, 7); dh_d = [2, 0, 2, 0, 2, 0, 1.15];
    baseBody = rigidBody('ArmBase'); baseJoint = rigidBodyJoint('BaseFix', 'fixed');
    setFixedTransform(baseJoint, [0 0 1 0; 0 1 0 0; -1 0 0 0; 0 0 0 1]); baseBody.Joint = baseJoint;
    addBody(robot, baseBody, robot.BaseName);
    
    prevLink = 'ArmBase'; 
    for i = 1:7
        linkName = sprintf('Link%d', i); body = rigidBody(linkName);
        joint = rigidBodyJoint(sprintf('Joint%d', i), 'revolute'); joint.PositionLimits = [-pi, pi];
        setFixedTransform(joint, [dh_a(i) dh_alpha(i) dh_d(i) 0], 'dh'); body.Joint = joint;
        if dh_d(i) ~= 0
            L = dh_d(i); safe_L = L - 0.32; 
            R_cyl = axang2rotm([1 0 0, -dh_alpha(i)]); p_cyl = R_cyl * [0; 0; -L/2]; 
            colCyl = collisionCylinder(0.15, safe_L); colCyl.Pose = [R_cyl, p_cyl; 0 0 0 1]; addCollision(body, colCyl);
        end
        addBody(robot, body, prevLink); prevLink = linkName;
    end
    if addTile
        tileBody = rigidBody('TilePayload'); tileJoint = rigidBodyJoint('TileFix', 'fixed'); tileBody.Joint = tileJoint;
        tileCol = collisionCylinder(1.0, 0.05); tileCol.Pose = trvec2tform([0, 0, 0.025]); addCollision(tileBody, tileCol);
        addBody(robot, tileBody, 'Link7');
    end
end

function [t, q, qd, qdd] = trajectoryGeneration(q_start, q_goal, options)
    T = options.totalTime; n_samples = options.numSamples; t = linspace(0, T, n_samples); tau = t / T;
    s = 10 * tau.^3 - 15 * tau.^4 + 6 * tau.^5; s_dot = (30 * tau.^2 - 60 * tau.^3 + 30 * tau.^4) / T;
    s_ddot = (60 * tau - 180 * tau.^2 + 120 * tau.^3) / (T^2);
    delta_q = q_goal - q_start; q = q_start + delta_q * s; qd = delta_q * s_dot; qdd = delta_q * s_ddot;
end