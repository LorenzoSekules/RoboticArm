s = tf('s');

%% 1. Creazione dei blocchi di guadagno (MIMO 3x3 + 4 SISO indipendenti)
% --- Proporzionale (Kp) ---
Kp_123 = tunableGain('Kp_123', 3, 3);
Kp_4   = tunableGain('Kp_4', 1, 1);
Kp_5   = tunableGain('Kp_5', 1, 1);
Kp_6   = tunableGain('Kp_6', 1, 1);
Kp_7   = tunableGain('Kp_7', 1, 1);
Kp     = blkdiag(Kp_123, Kp_4, Kp_5, Kp_6, Kp_7); % Matrice 7x7

% --- Integrale (Ki) ---
Ki_123 = tunableGain('Ki_123', 3, 3);
Ki_4   = tunableGain('Ki_4', 1, 1);
Ki_5   = tunableGain('Ki_5', 1, 1);
Ki_6   = tunableGain('Ki_6', 1, 1);
Ki_7   = tunableGain('Ki_7', 1, 1);
Ki     = blkdiag(Ki_123, Ki_4, Ki_5, Ki_6, Ki_7); % Matrice 7x7

% --- Derivativo (Kd) ---
Kd_123 = tunableGain('Kd_123', 3, 3);
Kd_4   = tunableGain('Kd_4', 1, 1);
Kd_5   = tunableGain('Kd_5', 1, 1);
Kd_6   = tunableGain('Kd_6', 1, 1);
Kd_7   = tunableGain('Kd_7', 1, 1);
Kd     = blkdiag(Kd_123, Kd_4, Kd_5, Kd_6, Kd_7); % Matrice 7x7

%% 2. Filtro Derivativo
% Dato che ricevi già y_dot in ingresso, il filtro è un semplice passa-basso
tau = 0.05; % Polo a 20 rad/s (modifica in base al rumore dei tuoi encoder)
Fd  = 1 / (tau * s + 1);

%% 3. Assemblaggio del Controllore K_all
% Input:  21 segnali ordinati come [y (7); int_e (7); y_dot (7)]
% Output: 7 segnali di coppia (u_arm)
K_all = [-Kp,  Ki,  -Kd * Fd]; % Dimensione: 7 uscite, 21 ingressi

%% 4. Chiusura dell'anello tramite LFT
% La funzione LFT collega automaticamente le ULTIME uscite di G_arm_isolated 
% (i tuoi 21 pid_arm_inputs) agli ingressi di K_all, e le uscite di K_all 
% agli ULTIMI ingressi di G (i tuoi 7 u_arm). 
% È il vantaggio di aver ordinato i vettori perfettamente!

CL0 = lft(G_arm_isolated, K_all);

%% 5. Ottimizzazione
% Se la tua G_arm_isolated contiene già i pesi di performance generalizzati (We, Wu):
% systune si comporterà come hinfstruct e minimizzerà la norma H_inf del sistema.

opt = systuneOptions('RandomStart', 3);
[CL_tuned, fSoft, info] = systune(CL0, opt);

% Estrazione dei valori finali
Kp_final = getBlockValue(CL_tuned, 'Kp_123');
Ki_final = getBlockValue(CL_tuned, 'Ki_123');
Kd_final = getBlockValue(CL_tuned, 'Kd_123');