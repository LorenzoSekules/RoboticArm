% DIMOSTRAZIONE ESTESA: Dinamica Vera, Trucco, Polyfit (Uniforme, Chebyshev, Lobatto)
%clear; clc; close all;

t_start = 0; t_end = 120;
t_vec = linspace(t_start, t_end, 200);

% Angolo di partenza e arrivo (es. da 0 a 180 gradi per enfatizzare la non-linearità)
q_start = 0;
q_end   = pi; 

% Parametro temporale normalizzato [0, 1]
tau = t_vec / t_end;
s5 = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;

% --- METODO A (Dinamica Vera): Traiettoria angolare -> Tangente ---
q_traj = q_start + (q_end - q_start) * s5;
tan_vera = tan(q_traj / 4);

% --- METODO B (Trucco Dottorando): Tangente agli estremi -> Traiettoria ---
tan_start = tan(q_start / 4);
tan_end   = tan(q_end / 4);
tan_trucco = tan_start + (tan_end - tan_start) * s5;

% --- METODO C (La Nostra Soluzione): Fit Polinomiale sulla Tangente Vera ---
% Estraiamo i coefficienti polinomiali (grado 5) direttamente dalla curva vera
p = polyfit(tau, tan_vera, 5);
% Ricostruiamo la curva usando ESCLUSIVAMENTE il polinomio (niente tan()!)
tan_poly_eq = polyval(p, tau);

% --- METODO D: Polyfit (Nodi di Chebyshev / Radici Ottimizzate) ---
N_punti = 6; % Grado 5 richiede 6 punti di interpolazione
k = 1:N_punti;
tau_cheb = 0.5 - 0.5 * cos((2*k - 1) * pi / (2 * N_punti));
q_cheb = q_start + (q_end - q_start) * (10*tau_cheb.^3 - 15*tau_cheb.^4 + 6*tau_cheb.^5);
p_cheb = polyfit(tau_cheb, tan(q_cheb/4), 5);
tan_poly_cheb = polyval(p_cheb, tau);

% --- METODO E: Polyfit (Nodi di Chebyshev-Lobatto / Ancoraggio Estremi) ---
k_lob = 1:N_punti;
tau_lob = 0.5 - 0.5 * cos((k_lob - 1) * pi / (N_punti - 1));
q_lob = q_start + (q_end - q_start) * (10*tau_lob.^3 - 15*tau_lob.^4 + 6*tau_lob.^5);
p_lob = polyfit(tau_lob, tan(q_lob/4), 5);
tan_poly_lob = polyval(p_lob, tau);

% --- PLOT ---
figure('Color', 'w', 'Name', 'Confronto Approssimazioni LFT');

subplot(2,1,1);
plot(t_vec, tan_vera, 'k', 'LineWidth', 4, 'DisplayName', 'Metodo A (Vera)');
hold on; grid on;
plot(t_vec, tan_trucco, 'r--', 'LineWidth', 2, 'DisplayName', 'Metodo B (Trucco)');
plot(t_vec, tan_poly_eq, 'y--', 'LineWidth', 2, 'DisplayName', 'Metodo C (Uniforme)');
plot(t_vec, tan_poly_cheb, 'b--', 'LineWidth', 2, 'DisplayName', 'Metodo D (Chebyshev)');
plot(t_vec, tan_poly_lob, 'g--', 'LineWidth', 2, 'DisplayName', 'Metodo E (Lobatto)');
legend('Location', 'northwest');
title('Traiettorie tan(q/4)');
ylabel('Ampiezza');

subplot(2,1,2);
plot(t_vec, abs(tan_vera - tan_trucco), 'r', 'LineWidth', 1.5, 'DisplayName', 'Errore B (Trucco)');
hold on; grid on;
plot(t_vec, abs(tan_vera - tan_poly_eq), 'y', 'LineWidth', 1.5, 'DisplayName', 'Errore C (Uniforme)');
plot(t_vec, abs(tan_vera - tan_poly_cheb), 'b', 'LineWidth', 1.5, 'DisplayName', 'Errore D (Chebyshev)');
plot(t_vec, abs(tan_vera - tan_poly_lob), 'g', 'LineWidth', 2.5, 'DisplayName', 'Errore E (Lobatto)');
% Indicatori grafici di ancoraggio per Lobatto agli estremi
plot(t_vec(1), abs(tan_vera(1) - tan_poly_lob(1)), 'ko', 'MarkerFaceColor', 'g', 'HandleVisibility', 'off');
plot(t_vec(end), abs(tan_vera(end) - tan_poly_lob(end)), 'ko', 'MarkerFaceColor', 'g', 'HandleVisibility', 'off');
legend('Location', 'northwest');
title('Errore Assoluto rispetto alla Dinamica Vera');
xlabel('Tempo [s]');
ylabel('Errore');

% --- ANALISI ERRORI ---
err_B = max(abs(tan_vera - tan_trucco));
err_C = max(abs(tan_vera - tan_poly_eq));
err_D = max(abs(tan_vera - tan_poly_cheb));
err_E = max(abs(tan_vera - tan_poly_lob));

fprintf('\n--- ANALISI DEGLI ERRORI MASSIMI ---\n');
fprintf('Errore Metodo B (Trucco)    : %.6e\n', err_B);
fprintf('Errore Metodo C (Uniforme)  : %.6e\n', err_C);
fprintf('Errore Metodo D (Chebyshev) : %.6e\n', err_D);
fprintf('Errore Metodo E (Lobatto)   : %.6e\n', err_E);
fprintf('Errore Estremi Lobatto      : %.6e (t=0) | %.6e (t=600)\n', ...
    abs(tan_vera(1) - tan_poly_lob(1)), abs(tan_vera(end) - tan_poly_lob(end)));

%%

function y = hornerEval(p,x)
% Evaluates a polynomial (coefficients p, highest degree first) at x
% using only +,*,^ so that x may be a UREAL-based expression.

    y = p(1);
    for k = 2:numel(p)
        y = y.*x + p(k);
    end

end