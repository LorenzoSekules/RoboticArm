%% PLOT (Focalizzato sui Task di Presa/Rilascio Tile - Multipli di 3)

joint_labels = {'q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6', 'q_7'};
colors = lines(7);

% Figure 1: Joint positions
figure('Color', 'w', 'Name', 'Joint Positions', 'NumberTitle', 'off');
hold on; grid on;
for j = 1:7
    plot(t_vec_slow, q_traj_slow(j, :), 'LineWidth', 1.4, 'Color', colors(j, :), 'DisplayName', joint_labels{j});
end
xlabel('Time [s]'); ylabel('q [rad]'); title('Joint Positions q(t) — Task Zone Evidenziate');
legend('Location', 'eastoutside');
xlim([t_vec_slow(1), t_vec_slow(end)]);
evidenziaTaskTile(T_new, t_stasi, t_vec_slow);


% Figure 2: Joint velocities
figure('Color', 'w', 'Name', 'Joint Velocities', 'NumberTitle', 'off');
hold on; grid on;
for j = 1:7
    plot(t_vec_slow, qd_traj_slow(j, :), 'LineWidth', 1.4, 'Color', colors(j, :), 'DisplayName', joint_labels{j});
end
xlabel('Time [s]'); ylabel('qdot [rad/s]'); title('Joint Velocities qdot(t)');
legend('Location', 'eastoutside');
xlim([t_vec_slow(1), t_vec_slow(end)]);
evidenziaTaskTile(T_new, t_stasi, t_vec_slow);


% Figure 3: Joint accelerations
figure('Color', 'w', 'Name', 'Joint Accelerations', 'NumberTitle', 'off');
hold on; grid on;
for j = 1:7
    plot(t_vec_slow, qdd_traj_slow(j, :), 'LineWidth', 1.4, 'Color', colors(j, :), 'DisplayName', joint_labels{j});
end
xlabel('Time [s]'); ylabel('qddot [rad/s^2]'); title('Joint Accelerations qddot(t)');
legend('Location', 'eastoutside');
xlim([t_vec_slow(1), t_vec_slow(end)]);
evidenziaTaskTile(T_new, t_stasi, t_vec_slow);


% Figure 4: Norm of Joint Velocities ||qdot(t)||
norm_qd = sqrt(sum(qd_traj_slow.^2, 1)); 
figure('Color', 'w', 'Name', 'Joint Velocity Norm', 'NumberTitle', 'off');
hold on; grid on;
plot(t_vec_slow, norm_qd, 'LineWidth', 1.5, 'Color', [0.07, 0.62, 0.45]);
xlabel('Time [s]'); ylabel('||qdot|| [rad/s]'); title('Norm of Joint Velocities ||qdot(t)||');
xlim([t_vec_slow(1), t_vec_slow(end)]);
evidenziaTaskTile(T_new, t_stasi, t_vec_slow);


%% --- FUNZIONE LOCALE DI EVIDENZIAZIONE (da incollare in fondo allo script) ---

function evidenziaTaskTile(durata_seg, durata_stasi, t_vec)
    yl = ylim(); 
    
    % I task nominali che vuoi colorare: 2, 5, 8, 11...
    task_da_evidenziare = 2 : 3 : floor(t_vec(end)/durata_seg); 

    for k = task_da_evidenziare
        
        % ---> LA FORMULA MAGICA: Quante stasi sono già passate prima di questo task?
        pause_accumulate = floor(k / 3); 
        
        t_inizio = (k - 1) * durata_seg + (pause_accumulate * durata_stasi);
        t_fine   = t_inizio + durata_seg;
        
        if t_inizio < t_vec(end)
            t_fine = min(t_fine, t_vec(end));
            
            p = patch([t_inizio, t_fine, t_fine, t_inizio], ...
                      [yl(1), yl(1), yl(2), yl(2)], ...
                      [1.0, 0.95, 0.82], 'FaceAlpha', 0.6, ...
                      'EdgeColor', [0.85, 0.72, 0.35], 'LineStyle', '--', 'LineWidth', 1.2, ...
                      'HandleVisibility', 'off');
            uistack(p, 'bottom');
            
            t_mid = (t_inizio + t_fine) / 2;
            text(t_mid, yl(1) + 0.95 * (yl(2) - yl(1)), sprintf('Tile S%d', k), ...
                'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold', ...
                'Color', [0.45 0.35 0.1], 'Clipping', 'on');
        end
    end
end