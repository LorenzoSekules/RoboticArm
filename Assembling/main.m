%% 1. Definizione del Rigid Body Tree (Aggiornato)
robot = robotics.RigidBodyTree("MaxNumBodies", 8, "DataFormat", "row");

% Corpo 0: Il cilindro fisso che sposta la base a [1.5, 0, 0]
body0 = robotics.RigidBody('body0');
jnt0 = robotics.Joint('jnt0','fixed');
% Traslazione lungo X di 1.5
setFixedTransform(jnt0, [1 0 0 1.5; 0 1 0 0; 0 0 1 0; 0 0 0 1]); 
body0.Joint = jnt0;
addBody(robot, body0, robot.BaseName);

% Definizione dei corpi e giunti mobili (1-7)
body1 = robotics.RigidBody('body1'); jnt1 = robotics.Joint('jnt1','revolute');
body2 = robotics.RigidBody('body2'); jnt2 = robotics.Joint('jnt2','revolute');
body3 = robotics.RigidBody('body3'); jnt3 = robotics.Joint('jnt3','revolute');
body4 = robotics.RigidBody('body4'); jnt4 = robotics.Joint('jnt4','revolute');
body5 = robotics.RigidBody('body5'); jnt5 = robotics.Joint('jnt5','revolute');
body6 = robotics.RigidBody('body6'); jnt6 = robotics.Joint('jnt6','revolute');
body7 = robotics.RigidBody('body7'); jnt7 = robotics.Joint('jnt7','revolute');

% Limiti dei giunti (in radianti)
jnt1.PositionLimits = [-170 170] * (pi/180);
jnt2.PositionLimits = [-120 120] * (pi/180);
jnt3.PositionLimits = [-170 170] * (pi/180);
jnt4.PositionLimits = [-120 120] * (pi/180);
jnt5.PositionLimits = [-170 170] * (pi/180);
jnt6.PositionLimits = [-120 120] * (pi/180);
jnt7.PositionLimits = [-175 175] * (pi/180);

% Nuove trasformazioni con offset = 0.5 per tutti
setFixedTransform(jnt1, [ 1  0  0 0;  0  1  0 0;   0  0  1 0.5; 0 0 0 1]);
setFixedTransform(jnt2, [-1  0  0 0;  0  0  1 0;   0  1  0 0.5; 0 0 0 1]);
setFixedTransform(jnt3, [-1  0  0 0;  0  0  1 0.5; 0  1  0 0;   0 0 0 1]);
setFixedTransform(jnt4, [ 1  0  0 0;  0  0 -1 0;   0  1  0 0.5; 0 0 0 1]);
setFixedTransform(jnt5, [-1  0  0 0;  0  0  1 0.5; 0  1  0 0;   0 0 0 1]);
setFixedTransform(jnt6, [ 1  0  0 0;  0  0 -1 0;   0  1  0 0.5; 0 0 0 1]);
setFixedTransform(jnt7, [-1  0  0 0;  0  0  1 0.5; 0  1  0 0;   0 0 0 1]);

% Assemblaggio dell'albero
body1.Joint = jnt1; addBody(robot, body1, 'body0');
body2.Joint = jnt2; addBody(robot, body2, 'body1');
body3.Joint = jnt3; addBody(robot, body3, 'body2');
body4.Joint = jnt4; addBody(robot, body4, 'body3');
body5.Joint = jnt5; addBody(robot, body5, 'body4');
body6.Joint = jnt6; addBody(robot, body6, 'body5');
body7.Joint = jnt7; addBody(robot, body7, 'body6');


%% 2. Generazione della Traiettoria Completa (Pose: Posizione + Orientamento)
% Definiamo la Posa di Partenza (Traslazione + Rotazione)
% Rotazione di -90 gradi su Y affinché l'asse Z locale punti verso -X globale
R_start = axang2rotm([0 1 0 -pi/2]); 
T_start = trvec2tform([0 0 0.5]) * rotm2tform(R_start);

% Definiamo la Posa di Arrivo
% Rotazione di 180 gradi su Y affinché l'asse Z locale punti verso -Z globale (basso)
R_end = axang2rotm([0 1 0 pi]); 
T_end = trvec2tform([3.5 0 -1.5]) * rotm2tform(R_end);

% Interpolazione fluida tra le due pose (posizione e orientamento insieme)
num_steps = 50;
tPts = [0 1];
tvec = linspace(0, 1, num_steps);
% transformtraj crea matrici 4x4 per ogni step temporale
T_traj = transformtraj(T_start, T_end, tPts, tvec); 


%% 3. Calcolo della Cinematica Inversa (IK) con pesi aggiornati
ik = inverseKinematics('RigidBodyTree', robot);

% Pesi: [Orientamento X, Y, Z, Posizione X, Y, Z]
% Ora sono tutti a 1: pretendiamo che il robot rispetti sia XYZ che l'inclinazione!
weights = [1 1 1 1 1 1]; 

q_history = zeros(num_steps, 7);
initialGuess = robot.homeConfiguration;

disp('Calcolo della cinematica inversa in corso...');
for i = 1:num_steps
    % Estraiamo la matrice 4x4 target per questo istante di tempo
    targetPose = T_traj(:, :, i);
    
    [q_sol, solInfo] = ik('body7', targetPose, weights, initialGuess);
    
    % CORREZIONE: q_sol è già un vettore riga grazie a "DataFormat", "row"
    q_history(i, :) = q_sol; 
    
    initialGuess = q_sol; 
end
disp('Calcolo completato!');

%% 4. Animazione 3D del Movimento
figure('Name', 'Animazione Traiettoria Robot', 'Position', [100, 100, 800, 600]);
show(robot, q_history(1,:)); % Mostra la posizione iniziale
hold on;
axis([-1 4 -2 2 -2 2]); % Imposta i limiti degli assi [Xmin Xmax Ymin Ymax Zmin Zmax]
grid on;

% Disegniamo un pallino rosso sul punto di arrivo
plot3(3.5, 0, -1.5, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

disp('Avvio animazione...');
for i = 1:num_steps
    % Aggiorna la figura con la configurazione dei giunti calcolata
    show(robot, q_history(i,:), 'PreservePlot', false, 'Frames', 'off');
    drawnow;
    pause(0.05); % Pausa per rendere l'animazione visibile
end