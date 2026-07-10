s=tf('s');
G=1/s^2;

kp = tunableGain('kp',1);
ki = tunableGain('ki',1);
kd = tunableGain('kd',1);

%Tf = 0.01;    % derivative filter
C = kp + ki/s + kd*s/(kd/(kp*10)*s+1);
C.u = 'e';
C.y = 't';

OL = linearize('trial');

CL0 = lft(OL,C);
CL0.u={'ref'};
CL0.y={'y','err','u'};

wm = 1;
W_Sens = makeweight(0.01,wm,2);
%W_Sens = 2*(s+0.05)^6/(s+0.5)^6;
% figure(1)
% hold on
% grid on
% bodemag(W_Sens)

Req_Sens = TuningGoal.Gain('ref','err',W_Sens);
Req_u    = TuningGoal.Gain('ref','u',100);
Req_con = TuningGoal.Gain('e','t',10);
Req_poles = TuningGoal.Poles(10^-3,0.7,inf);
Req_OS = TuningGoal.Overshoot('ref','y',0);
Req_step = TuningGoal.StepTracking('ref','y',1);
%% Tune
opt = systuneOptions( ...
    'Display','iter', ...
    'RandomStart',0, ...
    'MaxIter',200);

CL = systune(blkdiag(CL0,C),[],[Req_Sens,Req_poles,Req_u,Req_step],opt);

Kp = ss(CL.blocks.kp).D;
Kd = ss(CL.blocks.kd).D;
Ki = ss(CL.blocks.ki).D;


%% Extract tuned gains
showTunable(CL)

figure();
viewGoal(Req_Sens,CL)

figure();
viewGoal(Req_u,CL)

figure();
viewGoal(Req_con,CL)

figure();
viewGoal(Req_poles,CL)


figure();
viewGoal(Req_OS,CL)


figure();
viewGoal(Req_step,CL)

%%

% Update the controller with the tuned gains
C = Kp + Ki/s + Kd*s/(Kd/(Kp*10)*s + 1);
F= feedback(ss(C*G),1);
damp(F)

figure();
bodemag(F)

figure();
bodemag(G)
hold on
bodemag(C)
bodemag(G*C)
legend('G','C','L')
grid on



%% Disaccoppiamento

gen_names = @(prefix, n) arrayfun(@(x) sprintf('%s(%d)', prefix, x), 1:n, 'UniformOutput', false);

% --- CONTROL INPUTS (u) - Must be last for LFT ---
u_aocs  = gen_names('u_AOCS', 3);
u_arm   = gen_names('u_Arm', 7);

all_inputs = [u_aocs, u_arm];

% --- EXOGENOUS OUTPUTS (z) - Exposed for Performance Evaluation ---
q_aocs           = gen_names('q_AOCS', 3);
q_arm            = gen_names('q_Arm', 7);

all_outputs = [q_aocs, q_arm];


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
    
    sys_open_loop = ulinearize(sdtModel);
    sys_open_loop = minreal(sys_open_loop);

    % Force the I/O names to match string arrays based on Simulink port order
    sys_open_loop.u  = all_inputs;
    sys_open_loop.y = all_outputs;
    
    G_SDT_samples{i} = sys_open_loop;
end

% Stack into a Multi-Model Uncertain State-Space Array
G_array = stack(1, G_SDT_samples{:});

%%

G1 = G_SDT_samples{1}(all_outputs(1:3),all_inputs(1:3));
%% ===================================================================
%  RELATIVE GAIN ARRAY (RGA) — SDT System
%  Definizione: RGA(G) = G .* inv(G).'   [prodotto di Hadamard]
%  Interpretazione:
%    • RGA ≈ I  → sistema ben disaccoppiato
%    • |λ_ii| >> 1 → forte interazione, controllo difficile
%  RGA Number = Σ|λ_ij - δ_ij|  → 0 se perfettamente disaccoppiato
% ===================================================================

%% 1. Estrazione modello nominale (da USS)
if isa(G1, 'uss')
    G_nom = G1.NominalValue;
else
    G_nom = G1;
end
[n_out, n_in] = size(G_nom);
fprintf('Sistema: %d uscite × %d ingressi\n', n_out, n_in);
assert(n_out == n_in, 'RGA definita per sistemi quadrati (n_out == n_in).');


%% 2. RGA a bassa frequenza (gestione integratori)
G_dc = dcgain(G_nom);

if any(isinf(G_dc(:))) || any(isnan(G_dc(:)))
    omega_ref  = 1e-3;                                    % rad/s
    G_ref      = squeeze(freqresp(G_nom, omega_ref));
    label_ref  = sprintf('\\omega = 10^{-3} rad/s  (integratori presenti)');
    fprintf('[!] Integratori rilevati: RGA di riferimento a omega = %.0e rad/s\n', omega_ref);
else
    G_ref     = G_dc;
    label_ref = 'DC  (\omega = 0)';
    omega_ref = 0;
end

RGA_ref = real(G_ref .* inv(G_ref).');       % parte reale a freq. reale

fprintf('\n--- RGA a %s ---\n', strrep(label_ref,'\',''));
disp(array2table(round(RGA_ref,3), ...
     'RowNames', all_outputs(1:3), 'VariableNames', strrep(all_inputs(1:3),'(','_')));

%% Calcolo Indice di Niederlinski (NI)
det_G = det(G_ref);
prod_diag = prod(diag(G_ref));
NI = det_G / prod_diag;

fprintf('\n--- Indice di Niederlinski (NI) ---\n');
fprintf('NI = %.4f\n', NI);

if NI < 0
    fprintf('✗ ATTENZIONE: NI negativo. Il sistema decentralizzato diagonale è instabile con integratori!\n');
else
    fprintf('✓ OK: NI positivo. Il sistema può essere stabilizzato.\n');
end

%% 2. SVD della gain matrix di riferimento
[U, S, V] = svd(G_ref);
sigma = diag(S);                         % Valori singolari (ordine decrescente)
n_sv  = numel(sigma);
kappa = sigma(1) / sigma(end);           % Condition number

fprintf('\n=========================================\n');
fprintf('  SVD — Gain Matrix a %s\n', strrep(label_ref,'\',''));
fprintf('=========================================\n');
fprintf('  Valori Singolari:\n');
for k = 1:n_sv
    fprintf('    σ_%02d = %10.4e', k, sigma(k));
    if k == 1,    fprintf('  ← σ_max'); end
    if k == n_sv, fprintf('  ← σ_min'); end
    fprintf('\n');
end
fprintf('\n  Condition Number  κ = σ_max/σ_min = %.2f\n', kappa);

% --- Giudizio ---
if kappa < 10
    verdict = '✓ OTTIMO       — sistema ben condizionato, disaccoppiamento agevole';
    vcolor  = [0.0 0.55 0.25];
elseif kappa < 50
    verdict = '⚠ ACCETTABILE  — κ < 50, si può procedere con la RGA';
    vcolor  = [0.85 0.55 0.0];
else
    verdict = '✗ CRITICO      — κ ≥ 50, sistema mal condizionato, RGA inaffidabile';
    vcolor  = [0.80 0.10 0.10];
end
fprintf('\n  → %s\n', verdict);
fprintf('=========================================\n\n');

%% 3. Condition Number in FREQUENZA σ_max(ω)/σ_min(ω)
omega   = logspace(-3, 3, 400);
kappa_f = zeros(1, numel(omega));
smax_f  = zeros(1, numel(omega));
smin_f  = zeros(1, numel(omega));

for k = 1:numel(omega)
    sv_k     = svd(squeeze(freqresp(G_nom, omega(k))));
    smax_f(k) = sv_k(1);
    smin_f(k) = sv_k(end);
    kappa_f(k) = sv_k(1) / sv_k(end);
end

%% 4. Figure 1 — SVD statico: singular values + vettori singolari
figure('Name','SVD Gain Matrix','Color','w','Position',[80 80 1100 460]);

% --- Subplot A: Singular Values ---
ax1 = subplot(1,3,1);
bh = bar(1:n_sv, sigma, 'FaceColor','flat', 'EdgeColor','none', 'BarWidth', 0.7);
for k = 1:n_sv
    if k == n_sv
        bh.CData(k,:) = [0.85 0.20 0.15];  % σ_min in rosso
    else
        bh.CData(k,:) = [0.20 0.47 0.75];
    end
end
set(ax1,'YScale','log','XTick',1:n_sv,'FontSize',8);
xlabel('Indice  k',   'FontSize',11);
ylabel('\sigma_k',    'FontSize',12);
title('Valori Singolari (log)', 'FontSize',11);
grid on; box on;
text(n_sv*0.5, sigma(1)*0.3, ...
     sprintf('\\kappa = %.1f', kappa), ...
     'FontSize', 14, 'FontWeight', 'bold', ...
     'HorizontalAlignment','center', 'Color', vcolor);

% --- Subplot B: Left Singular Vectors |U| (output directions) ---
ax2 = subplot(1,3,2);
imagesc(abs(U)); colorbar; colormap(ax2, parula);
clim([0 1]);
title('|U| — Direzioni di Uscita', 'FontSize',11);
xlabel('Direzione Singolare', 'FontSize',10);
ylabel('Uscita  q_i',         'FontSize',10);
set(ax2,'YTick',1:n_out,'YTickLabel',all_outputs,'FontSize',7);
set(ax2,'XTick',1:n_sv);
axis tight; axis square;

% --- Subplot C: Right Singular Vectors |V| (input directions) ---
ax3 = subplot(1,3,3);
imagesc(abs(V)); colorbar; colormap(ax3, parula);
clim([0 1]);
title('|V| — Direzioni di Ingresso', 'FontSize',11);
xlabel('Direzione Singolare',  'FontSize',10);
ylabel('Ingresso  u_j',        'FontSize',10);
set(ax3,'YTick',1:n_in,'YTickLabel',all_inputs,'FontSize',7,'YTickLabelRotation',0);
set(ax3,'XTick',1:n_sv);
axis tight; axis square;

sgtitle(sprintf('SVD — Gain Matrix a %s  |  \\kappa = %.1f', label_ref, kappa), ...
        'FontSize', 13);

%% 5. Figure 2 — Condition Number in frequenza
figure('Name','Condition Number vs Frequency','Color','w','Position',[80 580 860 380]);
ax4 = axes;
fill([omega, fliplr(omega)], [smax_f, fliplr(smin_f)], ...
     [0.75 0.87 0.95], 'EdgeColor','none', 'FaceAlpha', 0.5); hold on;
semilogx(omega, smax_f,  'b-',  'LineWidth', 2, 'DisplayName','\sigma_{max}(\omega)');
semilogx(omega, smin_f,  'r--', 'LineWidth', 2, 'DisplayName','\sigma_{min}(\omega)');
set(ax4, 'YScale', 'log', 'XScale', 'log');
xlabel('Frequenza  [rad/s]', 'FontSize', 12);
ylabel('\sigma  [log]',      'FontSize', 12);
title('\sigma_{max} / \sigma_{min} — SDT System', 'FontSize', 13);
legend('Intervallo \sigma', '\sigma_{max}', '\sigma_{min}', ...
       'Location','best', 'FontSize',10);
grid on; box on;

% Condition number su asse destro
yyaxis right
semilogx(omega, kappa_f, 'k-', 'LineWidth', 1.5);
yline(50, 'm--', '\kappa = 50  (soglia)', 'LineWidth', 1.2, ...
      'LabelHorizontalAlignment','left', 'FontSize', 9);
ylabel('\kappa(\omega) = \sigma_{max}/\sigma_{min}', 'FontSize', 12);
set(gca,'YColor','k');

%% 6. Decisione automatica: si può procedere con la RGA?
fprintf('  Condition number a omega_ref  : κ = %.2f\n', kappa);
kappa_banda = kappa_f(omega >= 1e-2 & omega <= 1e0);
fprintf('  Condition number max in banda [0.01–1] rad/s: κ = %.2f\n', max(kappa_banda));

if kappa < 50 && max(kappa_banda) < 50
    fprintf('\n  ✓ SISTEMA DISACCOPPIABILE — Procedo con il calcolo della RGA.\n\n');
    PROCEED_RGA = true;
else
    fprintf('\n  ✗ ATTENZIONE — κ ≥ 50. Valutare pre-compensatore prima della RGA.\n\n');
    PROCEED_RGA = false;
end

%% 3. RGA in frequenza (sweep completo)
omega      = logspace(-3, 3, 400);           % [rad/s]
RGA_freq   = zeros(n_out, n_in, numel(omega));
RGA_number = zeros(1, numel(omega));

for k = 1:numel(omega)
    Gjw             = squeeze(freqresp(G_nom, omega(k)));
    Lk              = Gjw .* inv(Gjw).';
    RGA_freq(:,:,k) = Lk;
    RGA_number(k)   = sum(sum(abs(Lk - eye(n_out))));    % RGA number
end

%% 4. Plot — RGA Number vs frequenza
figure('Name','RGA Number','Color','w','Position',[100 100 820 380]);
semilogx(omega, RGA_number, 'b-', 'LineWidth', 2.5); hold on;
semilogx(omega, zeros(size(omega)), 'k--', 'LineWidth', 1);
xlabel('Frequenza  [rad/s]', 'FontSize', 12);
ylabel('RGA Number  \Sigma_{ij}|\lambda_{ij} - \delta_{ij}|', 'FontSize', 12);
title('RGA Number — SDT System', 'FontSize', 13);
grid on; box on;
% Annotazione alla banda di controllo tipica
xregion(1e-2, 1e0, 'FaceColor', [0.8 0.9 1], 'FaceAlpha', 0.4);
text(0.1, max(RGA_number)*0.85, 'Banda di interesse', ...
     'FontSize', 9, 'Color', [0 0 0.6]);

%% 5. Plot — Heatmap |RGA| di riferimento
figure('Name','RGA Heatmap','Color','w','Position',[100 520 680 560]);
imagesc((RGA_ref));
colorbar; colormap(copper);
clim([min(RGA_ref(:)), max(abs(RGA_ref(:)))]);
xlabel('Ingresso  u_j', 'FontSize', 11);
ylabel('Uscita  q_i',   'FontSize', 11);
title(['|RGA| a  ', label_ref], 'FontSize', 12);
set(gca, 'XTick', 1:n_in,  'XTickLabel', all_inputs,  ...
         'XTickLabelRotation', 45, 'FontSize', 7.5);
set(gca, 'YTick', 1:n_out, 'YTickLabel', all_outputs, 'FontSize', 7.5);
% Annotazioni numeriche sulle celle
for r = 1:n_out
    for c = 1:n_in
        val = (RGA_ref(r,c));
        clr = 'w';
        if val < 0.05 * max(abs(RGA_ref(:))), clr = 'k'; end
        text(c, r, sprintf('%.2f', val), ...
            'HorizontalAlignment','center','FontSize',6,'Color', clr);
    end
end
axis square; axis tight;

%% 6. Plot — Elementi DIAGONALI |λ_jj| vs frequenza
figure('Name','RGA Diagonal','Color','w','Position',[950 100 820 500]);
hold on; grid on; box on;
cols = lines(n_out);
for j = 1:n_out
    semilogx(omega, abs(squeeze(RGA_freq(j,j,:))), ...
        'Color', cols(j,:), 'LineWidth', 1.6, 'DisplayName', all_outputs{j});
end
yline(1, 'k--', '\lambda_{jj}=1  (ideale)', 'LineWidth', 1.2, ...
      'LabelHorizontalAlignment','left');
set(gca, 'XScale','log');
xlabel('Frequenza  [rad/s]', 'FontSize', 12);
ylabel('|\lambda_{jj}(\omega)|',  'FontSize', 12);
title('Elementi Diagonali RGA — SDT System', 'FontSize', 13);
legend('Location','bestoutside','FontSize',7,'Interpreter','none');
ylim([0, min(5, prctile(abs(RGA_freq(:)), 99))]);

%% 7. Sommario testuale
fprintf('\n========== SOMMARIO RGA ==========\n');
fprintf('Canali con |lambda_ii| > 2 a bassa freq. (forte interazione):\n');
for j = 1:n_out
    if abs(RGA_ref(j,j)) > 0.5
        fprintf('  → %s / %s   |λ_jj| = %.2f\n', ...
                all_outputs{j}, all_inputs{j}, abs(RGA_ref(j,j)));
    end
end
fprintf('RGA Number a omega=0.1 rad/s: %.3f\n', ...
        RGA_number(find(omega >= 0.1, 1)));
fprintf('RGA Number minimo: %.3f @ omega = %.3f rad/s\n', ...
        min(RGA_number), omega(RGA_number == min(RGA_number)));
fprintf('===================================\n');




%% %% FUNCTIONS
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

