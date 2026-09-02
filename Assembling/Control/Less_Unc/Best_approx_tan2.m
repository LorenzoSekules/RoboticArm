% DIMOSTRAZIONE INVERSA: Ricaduta dell'Errore LFT sugli Angoli Fisici
clear; clc; close all;

t_start = 0; t_end = 120;
t_vec = linspace(t_start, t_end, 500);

% Angolo di partenza e arrivo (0 -> 180 gradi)
q_start = 0; q_end = pi; 
tau = t_vec / t_end;
s5 = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;

% --- GENERAZIONE NEL DOMINIO DELLA TANGENTE ---
% Metodo A: Vera Traiettoria
q_vera_rad = q_start + (q_end - q_start) * s5;
tan_vera = tan(q_vera_rad / 4);

% Metodo B: Trucco
tan_trucco = tan(q_start/4) + (tan(q_end/4) - tan(q_start/4)) * s5;

% Metodo C: Uniforme
N_punti = 6;
tau_eq = linspace(0, 1, N_punti);
q_eq = q_start + (q_end - q_start) * (10*tau_eq.^3 - 15*tau_eq.^4 + 6*tau_eq.^5);
p_eq = polyfit(tau_eq, tan(q_eq/4), 5);
tan_poly_eq = polyval(p_eq, tau);

% Metodo D: Chebyshev
k = 1:N_punti;
tau_cheb = 0.5 - 0.5 * cos((2*k - 1) * pi / (2 * N_punti));
q_cheb = q_start + (q_end - q_start) * (10*tau_cheb.^3 - 15*tau_cheb.^4 + 6*tau_cheb.^5);
p_cheb = polyfit(tau_cheb, tan(q_cheb/4), 5);
tan_poly_cheb = polyval(p_cheb, tau);

% Metodo E: Lobatto
k_lob = 1:N_punti;
tau_lob = 0.5 - 0.5 * cos((k_lob - 1) * pi / (N_punti - 1));
q_lob = q_start + (q_end - q_start) * (10*tau_lob.^3 - 15*tau_lob.^4 + 6*tau_lob.^5);
p_lob = polyfit(tau_lob, tan(q_lob/4), 5);
tan_poly_lob = polyval(p_lob, tau);

% --- TRASFORMAZIONE INVERSA NEGLI ANGOLI REALI [GRADI] ---
% q = 4 * atan(tan_value)
q_vera_deg = (4 * atan(tan_vera)) * (180/pi);
q_trucco_deg = (4 * atan(tan_trucco)) * (180/pi);
q_poly_eq_deg = (4 * atan(tan_poly_eq)) * (180/pi);
q_poly_cheb_deg = (4 * atan(tan_poly_cheb)) * (180/pi);
q_poly_lob_deg = (4 * atan(tan_poly_lob)) * (180/pi);

% --- PLOT ---
figure('Color', 'w', 'Name', 'Impatto Angolare delle Approssimazioni LFT');

subplot(2,1,1);
plot(t_vec, q_vera_deg, 'k', 'LineWidth', 4, 'DisplayName', 'Metodo A (Vera)');
hold on; grid on;
plot(t_vec, q_trucco_deg, 'r--', 'LineWidth', 2, 'DisplayName', 'Metodo B (Trucco)');
plot(t_vec, q_poly_eq_deg, 'y--', 'LineWidth', 2, 'DisplayName', 'Metodo C (Uniforme)');
plot(t_vec, q_poly_lob_deg, 'g--', 'LineWidth', 2, 'DisplayName', 'Metodo E (Lobatto)');
legend('Location', 'northwest');
title('Traiettorie Angolari Fisiche [Gradi]');
ylabel('Angolo [°]');

subplot(2,1,2);
plot(t_vec, abs(q_vera_deg - q_trucco_deg), 'r', 'LineWidth', 1.5, 'DisplayName', 'Errore B (Trucco)');
hold on; grid on;
plot(t_vec, abs(q_vera_deg - q_poly_eq_deg), 'y', 'LineWidth', 1.5, 'DisplayName', 'Errore C (Uniforme)');
plot(t_vec, abs(q_vera_deg - q_poly_cheb_deg), 'b', 'LineWidth', 1.5, 'DisplayName', 'Errore D (Chebyshev)');
plot(t_vec, abs(q_vera_deg - q_poly_lob_deg), 'g', 'LineWidth', 2.5, 'DisplayName', 'Errore E (Lobatto)');
plot(t_vec(1), abs(q_vera_deg(1) - q_poly_lob_deg(1)), 'ko', 'MarkerFaceColor', 'g', 'HandleVisibility', 'off');
plot(t_vec(end), abs(q_vera_deg(end) - q_poly_lob_deg(end)), 'ko', 'MarkerFaceColor', 'g', 'HandleVisibility', 'off');
legend('Location', 'northwest');
title('Errore Fisico Assoluto [Gradi]');
xlabel('Tempo [s]');
ylabel('Errore [°]');

% --- ANALISI ERRORI FISICI ---
fprintf('\n--- ERRORE MASSIMO SUI GIUNTI [GRADI] ---\n');
fprintf('Errore Metodo B (Trucco)    : %.4f°\n', max(abs(q_vera_deg - q_trucco_deg)));
fprintf('Errore Metodo C (Uniforme)  : %.4f°\n', max(abs(q_vera_deg - q_poly_eq_deg)));
fprintf('Errore Metodo D (Chebyshev) : %.4f°\n', max(abs(q_vera_deg - q_poly_cheb_deg)));
fprintf('Errore Metodo E (Lobatto)   : %.4f°\n', max(abs(q_vera_deg - q_poly_lob_deg)));