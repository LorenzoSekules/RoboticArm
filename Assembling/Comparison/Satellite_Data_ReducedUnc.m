%% Satellite_Data_ReducedUnc
% Variante di Satellite_Data.m in cui TUTTE le proprieta' di massa/inerzia
% (corpo centrale, container, mattoncini, link del braccio, tile) sono
% rese NOMINALI (double, niente ureal). L'unica incertezza mantenuta e':
%   - il boom flessibile (via Nastran2ROFS / parametri boom.*), invariato
%   - gli angoli di giunto del braccio robotico (robot.config{1..7})
%
% Scopo: costruire un uss "leggero" da usare per sintesi/robust control,
% senza portarsi dietro decine di parametri incerti che nella pratica
% (vedi Comparison2.m) venivano comunque collassati a NominalValue.

%% Main Body Data

% define main body dimensions (meters)
main_body_x = 2.2;   % length along x-axis
main_body_y = 3;     % length along y-axis
main_body_z = 5;     % length along z-axis

m_B     = 4000;                 % [kg] Mass central body (nominale, no ureal)
m_B_SIM = m_B;                  % [kg] Mass central body (used in simscape)

J_B_Bxx = m_B_SIM/12*(main_body_y^2+main_body_z^2); % [kg*m^2]
J_B_Byy = m_B_SIM/12*(main_body_x^2+main_body_z^2); % [kg*m^2]
J_B_Bzz = m_B_SIM/12*(main_body_y^2+main_body_x^2); % [kg*m^2]
J_B_Bxy = 0;  % [kg*m^2] Cross moment of inertia (xy)
J_B_Byz = 0;  % [kg*m^2] Cross moment of inertia (yz)
J_B_Bxz = 0;  % [kg*m^2] Cross moment of inertia (xz)

J_B_B = [J_B_Bxx J_B_Bxy J_B_Bxz;...
        J_B_Bxy J_B_Byy J_B_Byz;...
        J_B_Bxz J_B_Byz J_B_Bzz];  % Moment of inertia of the central body B [kg*m^2]
                        % Note the order in Simscape for product of inertia is given as
                        % J_B_B(2,3) J_B_B(1,3) J_B_B(1,2)

J_B_B_SIM = J_B_B; % gia' nominale

% CoM variation: portata a zero (nessuna incertezza sul CoM del main body)
CoM_variation_x = 0;
CoM_variation_y = 0;
CoM_variation_z = 0;

CoM_Variation_simscape = [CoM_variation_x; CoM_variation_y; CoM_variation_z]; % Variation of CoM used in the Simscape model

%% Container Data

Container_y = sqrt(3)+0.5;
Container_x = 1.5;
Container_z = sqrt(3)+0.5;

m_C     = 100;      % [kg] Mass central body (nominale)
m_C_SIM = m_C;       % [kg] Mass central body (used in simscape)

J_C_Cxx = m_C_SIM/12*(Container_y^2+Container_z^2); % [kg*m^2]
J_C_Cyy = m_C_SIM/12*(Container_x^2+Container_z^2); % [kg*m^2]
J_C_Czz = m_C_SIM/12*(Container_y^2+Container_x^2); % [kg*m^2]
J_C_Cxy = 0;  % [kg*m^2]
J_C_Cyz = 0;  % [kg*m^2]
J_C_Cxz = 0;  % [kg*m^2]

J_C_C = [J_C_Cxx J_C_Cxy J_C_Cxz;...
        J_C_Cxy J_C_Cyy J_C_Cyz;...
        J_C_Cxz J_C_Cyz J_C_Czz];  % [kg*m^2]
                        % Note the order in Simscape for product of inertia is given as
                        % J_C_C(2,3) J_C_C(1,3) J_C_C(1,2)

J_C_C_SIM = J_C_C; % gia' nominale

%%  Little Bricks Data

Brick_y = 0.2;
Brick_x = 0.2;
Brick_z = 0.4;

m_P     = 50;       % [kg] Mass central body (nominale)
m_P_SIM = m_P;       % [kg] Mass central body (used in simscape)

J_P_Pxx = m_P_SIM/12*(Brick_y^2+Brick_z^2); % [kg*m^2]
J_P_Pyy = m_P_SIM/12*(Brick_x^2+Brick_z^2); % [kg*m^2]
J_P_Pzz = m_P_SIM/12*(Brick_y^2+Brick_x^2); % [kg*m^2]
J_P_Pxy = 0;  % [kg*m^2]
J_P_Pyz = 0;  % [kg*m^2]
J_P_Pxz = 0;  % [kg*m^2]

J_P_P = [J_P_Pxx J_P_Pxy J_P_Pxz;...
        J_P_Pxy J_P_Pyy J_P_Pyz;...
        J_P_Pxz J_P_Pyz J_P_Pzz];  % [kg*m^2]
                        % Note the order in Simscape for product of inertia is given as
                        % J_P_P(2,3) J_P_P(1,3) J_P_P(1,2)

J_P_P_SIM = J_P_P; % gia' nominale

%% Flex beam Nastran  (UNICA incertezza "strutturale" mantenuta, insieme ai giunti)

% FEM model name
f06_boom='boom';
bdf_boom='boom';

% Interface points
boom.pointP  =       1;          % Attachment node of boom (grid point ID on bdf file)
boom.pointC =   11;         % Not used since we use a 1-port approach for the boom (tip node)
boom.damping_ratio =  0.003;  % Common damping ratio
boom.n_modes =    10;         % Number of modes for each boom
unc_freq_boom =   10;         % common uncertain (percentage) on natural frequency
boom.n_unc = 4;               % Number of modes considered uncertain

boom.MPCunc = 0;              % Uncertaintiy on the modal participation factor of the solar arrays
boom.n_MPCunc = 0;              % Number of uncertaintiy on the modal participation factor of the solar arrays

% Extraction of ROM matrix for Simscape Multibody Flex. Reduced order model
[coord_boom,Mrofs_boom,Krofs_boom,Drofs_boom,flagFatal]=Nastran2ROFS(strcat(f06_boom,'.f06'),strcat(bdf_boom,'.bdf'),boom.damping_ratio,boom.pointP,boom.pointC,boom.n_modes);

%% Robot Parameters
% Standard DH matrix for link i:
%   Ai = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)
%
% The homogeneous transform encodes both orientation and position:
%   T = [R p;
%        0 1]
% where R in SO(3), p in R^3.
robot = struct();
robot.nJoints = 7;
robot.alpha = [-pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, 0];
robot.a = zeros(1, 7);
robot.d = [2, 0, 2, 0, 2, 0, 1.15];
robot.baseT = [0 0 1 0; 0 1 0 0; -1 0 0 0; 0 0 0 1];
robot.jointLimits = repmat([-pi, pi], 7, 1);
robot.radius = 0.15;

% Effective orbital-arm density: lightweight CFRP structure plus metallic joints.
rho_arm = 1650; % [kg/m^3]

% Only the non-zero links are modeled as cylinders: 1, 3, 5, and 7.
activeLinks = find(robot.d ~= 0);
robot.linkLength = robot.d(activeLinks);
robot.linkRadius = robot.radius * ones(1, numel(activeLinks)); % [m]

% Masse/inerzie dei link: ORA NOMINALI (niente ureal). Struttura mantenuta
% identica per compatibilita' con il resto del codice/modello Simulink.
robot.linkMass = cell(1, numel(activeLinks));
robot.linkInertia = cell(1, numel(activeLinks));

m_RA_SIM = zeros(1,numel(activeLinks));
J_RA_SIM = zeros(3,3,numel(activeLinks));

for i = 1:numel(activeLinks)
        idx = activeLinks(i); % Actual robot.d index: 1, 3, 5, 7
        L_cyl = robot.linkLength(i);
        r_cyl = robot.linkRadius(i);

        % Nominal mass from equivalent cylindrical volume
        m_link = rho_arm * pi * r_cyl^2 * L_cyl;
        robot.linkMass{i} = m_link;
        m_RA_SIM(i) = m_link;

        J_xx = (m_RA_SIM(i) / 12) * (3 * r_cyl^2 + L_cyl^2);
        J_yy = (m_RA_SIM(i) / 12) * (3 * r_cyl^2 + L_cyl^2);
        J_zz = (m_RA_SIM(i) / 2)  * (r_cyl^2);

        % Cross moments of inertia nominalmente nulli (niente ureal)
        J_xy = 0;
        J_yz = 0;
        J_xz = 0;

        robot.linkInertia{i} = [J_xx J_xy J_xz;...
                                J_xy J_yy J_yz;...
                                J_xz J_yz J_zz];

        J_RA_SIM(:,:,i) = robot.linkInertia{i};
end

q_start = q_traj(:, 1);
q_min_traj = min(q_traj, [], 2);
q_max_traj = max(q_traj, [], 2);

% Initial angular configuration of the RA using the tan(theta/4) formalism.
% QUESTA E' L'ALTRA incertezza mantenuta (ureal sui 7 angoli di giunto),
% invariata rispetto allo script originale.
robot.config={ureal('Q_1' ,q_traj(1,1),'Range',[q_min_traj(1) q_max_traj(1)],'AutoSimplify','full')...
               ureal('Q_2' ,q_traj(2,1),'Range',[q_min_traj(2) q_max_traj(2)],'AutoSimplify','full')...
               ureal('Q_3' ,q_traj(3,1),'Range',[q_min_traj(3) q_max_traj(3)],'AutoSimplify','full')...
               ureal('Q_4' ,q_traj(4,1),'Range',[q_min_traj(4) q_max_traj(4)],'AutoSimplify','full')...
               ureal('Q_5' ,q_traj(5,1),'Range',[q_min_traj(5) q_max_traj(5)],'AutoSimplify','full')...
               ureal('Q_6' ,q_traj(6,1),'Range',[q_min_traj(6)-1e-3 q_max_traj(6)],'AutoSimplify','full')...
               ureal('Q_7' ,q_traj(7,1),'Range',[q_min_traj(7) q_max_traj(7)],'AutoSimplify','full')}; % (rad)

robot.config_SIM = [usubs(robot.config{1},'Q_1',robot.config{1}.NominalValue), ...
                    usubs(robot.config{2},'Q_2',robot.config{2}.NominalValue), ...
                    usubs(robot.config{3},'Q_3',robot.config{3}.NominalValue), ...
                    usubs(robot.config{4},'Q_4',robot.config{4}.NominalValue), ...
                    usubs(robot.config{5},'Q_5',robot.config{5}.NominalValue), ...
                    usubs(robot.config{6},'Q_6',robot.config{6}.NominalValue), ...
                    usubs(robot.config{7},'Q_7',robot.config{7}.NominalValue)];

%% Tile

% --- GEOMETRY AND MATERIAL PROPERTIES ---
rho = 81.2;          % [kg/m^3] Volumetric density of the reflectarray panel
Tile_s = 1;        % [m] Side length of the regular hexagon
Tile_z = 0.05;     % [m] Thickness/Height of the panel

% Calculate Volume of a regular hexagonal prism: V = (3*sqrt(3)/2) * s^2 * h
Volume = (3 * sqrt(3) / 2) * (Tile_s^2) * Tile_z;

% Calculate Nominal Mass
m_nominal = rho * Volume;

% --- MASSA (nominale, niente ureal) ---
m_T     = m_nominal;   % [kg]
m_T_SIM = m_T;          % [kg] (used in simscape)

% --- MOMENTI D'INERZIA (nominali) ---
J_T_Txx = m_T_SIM * ( (5/24)*(Tile_s^2) + (1/12)*(Tile_z^2) ); % [kg*m^2]
J_T_Tyy = m_T_SIM * ( (5/24)*(Tile_s^2) + (1/12)*(Tile_z^2) ); % [kg*m^2]
J_T_Tzz = m_T_SIM * (5/12)*(Tile_s^2);                          % [kg*m^2]

% --- CROSS MOMENTI D'INERZIA (nominalmente nulli per esagono regolare) ---
J_T_Txy = 0;  % [kg*m^2]
J_T_Tyz = 0;  % [kg*m^2]
J_T_Txz = 0;  % [kg*m^2]

% --- INERTIA TENSOR MATRIX ---
J_T_T = [J_T_Txx J_T_Txy J_T_Txz;...
         J_T_Txy J_T_Tyy J_T_Tyz;...
         J_T_Txz J_T_Tyz J_T_Tzz];  % [kg*m^2]
                                    % Note the order in Simscape for product of inertia is given as
                                    % J_T_T(2,3) J_T_T(1,3) J_T_T(1,2)

% --- SIMSCAPE INERTIA EVALUATION ---
J_T_T_SIM = J_T_T; % gia' nominale

%% Tiles (contact/stiffness parameters - invariati, non erano ureal)

K_tile_force = 2000/0.015/5000;
C_tile_force = 2*5*sqrt(K_tile_force/1*2.598*0.2*100);

K_tile_torque = 250/deg2rad(10)/1000;
C_tile_torque = 2*sqrt(K_tile_torque)*0.7;
