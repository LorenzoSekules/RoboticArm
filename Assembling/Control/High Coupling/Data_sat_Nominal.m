%% --- GESTIONE INCERTEZZE (Toggles) ---
% Se queste variabili non sono definite nel main, imposta i default qui:
if ~exist('UNC_MAIN_BODY', 'var')
    UNC_MAIN_BODY = false;
end
if ~exist('UNC_CONTAINER', 'var')
    UNC_CONTAINER = false;
end
if ~exist('UNC_BRICK', 'var')
    UNC_BRICK = false;
end
if ~exist('UNC_ROBOT_LINKS', 'var')
    UNC_ROBOT_LINKS = false;
end
if ~exist('UNC_TILE', 'var')
    UNC_TILE = false;
end
if ~exist('UNC_BOOM', 'var')
    UNC_BOOM = true;
end
if ~exist('UNC_JOINTS', 'var')
    UNC_JOINTS = true;
end

%% Main Body Data
main_body_x = 2.2;   % length along x-axis
main_body_y = 3;     % length along y-axis
main_body_z = 5;     % length along z-axis

m_B_SIM = 4000;      % [kg] Nominal Mass (used in simscape)
J_B_B_SIM = [m_B_SIM/12*(main_body_y^2+main_body_z^2), 0, 0; ...
             0, m_B_SIM/12*(main_body_x^2+main_body_z^2), 0; ...
             0, 0, m_B_SIM/12*(main_body_y^2+main_body_x^2)];
CoM_Variation_simscape = [0; 0; 0]; 

if UNC_MAIN_BODY
    m_B = ureal('m_B', m_B_SIM, 'percent', 20);
    J_B_Bxx = ureal('J_B_Bxx', J_B_B_SIM(1,1), 'percent', 20);
    J_B_Byy = ureal('J_B_Byy', J_B_B_SIM(2,2), 'percent', 20);
    J_B_Bzz = ureal('J_B_Bzz', J_B_B_SIM(3,3), 'percent', 20);
    J_B_Bxy = ureal('J_B_Bxy', 0, 'PlusMinus', [-1 1]);
    J_B_Byz = ureal('J_B_Byz', 0, 'PlusMinus', [-1 1]);
    J_B_Bxz = ureal('J_B_Bxz', 0, 'PlusMinus', [-1 1]);
    
    J_B_B = [J_B_Bxx J_B_Bxy J_B_Bxz; ...
             J_B_Bxy J_B_Byy J_B_Byz; ...
             J_B_Bxz J_B_Byz J_B_Bzz];
         
    CoM_variation_x = ureal('CoMx', 0, "PlusMinus", [-10e-3 10e-3]); 
    CoM_variation_y = ureal('CoMy', 0, "PlusMinus", [-10e-3 10e-3]);
    CoM_variation_z = ureal('CoMz', 0, "PlusMinus", [-20e-3 20e-3]);
else
    m_B = m_B_SIM;
    J_B_B = J_B_B_SIM;
    CoM_variation_x = 0;
    CoM_variation_y = 0;
    CoM_variation_z = 0;
end

%% Container Data
Container_y = sqrt(3)+0.5;
Container_x = 1.5;
Container_z = sqrt(3)+0.5;

m_C_SIM = 100;  % [kg] Nominal Mass (used in simscape)
J_C_C_SIM = [m_C_SIM/12*(Container_y^2+Container_z^2), 0, 0; ...
             0, m_C_SIM/12*(Container_x^2+Container_z^2), 0; ...
             0, 0, m_C_SIM/12*(Container_y^2+Container_x^2)];

if UNC_CONTAINER
    m_C = ureal('m_C', m_C_SIM, 'percent', 10);
    J_C_Cxx = ureal('J_C_Cxx', J_C_C_SIM(1,1), 'percent', 10);
    J_C_Cyy = ureal('J_C_Cyy', J_C_C_SIM(2,2), 'percent', 10);
    J_C_Czz = ureal('J_C_Czz', J_C_C_SIM(3,3), 'percent', 10);
    J_C_Cxy = ureal('J_C_Cxy', 0, 'PlusMinus', [-1 1]);
    J_C_Cyz = ureal('J_C_Cyz', 0, 'PlusMinus', [-1 1]);
    J_C_Cxz = ureal('J_C_Cxz', 0, 'PlusMinus', [-1 1]);
    
    J_C_C = [J_C_Cxx J_C_Cxy J_C_Cxz; ...
             J_C_Cxy J_C_Cyy J_C_Cyz; ...
             J_C_Cxz J_C_Cyz J_C_Czz];
else
    m_C = m_C_SIM;
    J_C_C = J_C_C_SIM;
end

%%  Little Bricks Data
Brick_y = 0.2;
Brick_x = 0.2;
Brick_z = 0.4;

m_P_SIM = 50;  % [kg] Nominal Mass (used in simscape)
J_P_P_SIM = [m_P_SIM/12*(Brick_y^2+Brick_z^2), 0, 0; ...
             0, m_P_SIM/12*(Brick_x^2+Brick_z^2), 0; ...
             0, 0, m_P_SIM/12*(Brick_y^2+Brick_x^2)];

if UNC_BRICK
    m_P = ureal('m_P', m_P_SIM, 'percent', 2);
    J_P_Pxx = ureal('J_P_Pxx', J_P_P_SIM(1,1), 'percent', 2);
    J_P_Pyy = ureal('J_P_Pyy', J_P_P_SIM(2,2), 'percent', 2);
    J_P_Pzz = ureal('J_P_Pzz', J_P_P_SIM(3,3), 'percent', 2);
    J_P_Pxy = ureal('J_P_Pxy', 0, 'PlusMinus', [-1 1]*0.1);
    J_P_Pyz = ureal('J_P_Pyz', 0, 'PlusMinus', [-1 1]*0.1);
    J_P_Pxz = ureal('J_P_Pxz', 0, 'PlusMinus', [-1 1]*0.1);
    
    J_P_P = [J_P_Pxx J_P_Pxy J_P_Pxz; ...
             J_P_Pxy J_P_Pyy J_P_Pyz; ...
             J_P_Pxz J_P_Pyz J_P_Pzz];
else
    m_P = m_P_SIM;
    J_P_P = J_P_P_SIM;
end

%% Flex beam Nastran
f06_boom='boom';
bdf_boom='boom';

boom.pointP  = 1;          
boom.pointC  = 11;         
boom.damping_ratio = 0.003;  
boom.n_modes = 10;         
boom.MPCunc = 0;              
boom.n_MPCunc = 0; 

if UNC_BOOM
    unc_freq_boom = 10;   % Incertezza [%]
    boom.n_unc = 4;            % Numero di modi incerti
else
    unc_freq_boom = 0;
    boom.n_unc = 0;
end

[coord_boom,Mrofs_boom,Krofs_boom,Drofs_boom,flagFatal]=Nastran2ROFS(strcat(f06_boom,'.f06'),strcat(bdf_boom,'.bdf'),boom.damping_ratio,boom.pointP,boom.pointC,boom.n_modes); 

%% Robot Parameters
robot = struct();
robot.nJoints = 7;
robot.alpha = [-pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, 0];
robot.a = zeros(1, 7);
robot.d = [2, 0, 2, 0, 2, 0, 1.15];
robot.baseT = [0 0 1 0; 0 1 0 0; -1 0 0 0; 0 0 0 1];
robot.jointLimits = repmat([-pi, pi], 7, 1);
robot.radius = 0.15;
rho_arm = 1650; % [kg/m^3]

activeLinks = find(robot.d ~= 0);
robot.linkLength = robot.d(activeLinks);
robot.linkRadius = robot.radius * ones(1, numel(activeLinks)); 

robot.linkMass = cell(1, numel(activeLinks));
robot.linkInertia = cell(1, numel(activeLinks));
m_RA_SIM = zeros(1,numel(activeLinks));
J_RA_SIM = zeros(3,3,numel(activeLinks));

for j = 1:numel(activeLinks)
    idx = activeLinks(j); 
    L_cyl = robot.linkLength(j);
    r_cyl = robot.linkRadius(j);
    
    % Nominal mass and inertia
    m_nom = rho_arm * pi * r_cyl^2 * L_cyl;
    m_RA_SIM(j) = m_nom;
    
    J_nom_xx = (m_nom / 12) * (3 * r_cyl^2 + L_cyl^2);
    J_nom_yy = J_nom_xx;
    J_nom_zz = (m_nom / 2) * (r_cyl^2);
    
    J_RA_SIM(:,:,j) = [J_nom_xx, 0, 0; 0, J_nom_yy, 0; 0, 0, J_nom_zz];

    if UNC_ROBOT_LINKS
        robot.linkMass{j} = ureal(sprintf('m_link%d', idx), m_nom, 'percent', 5);
        
        J_xx = ureal(sprintf('J_xx_link%d', idx), J_nom_xx, 'percent', 5);
        J_yy = ureal(sprintf('J_yy_link%d', idx), J_nom_yy, 'percent', 5);
        J_zz = ureal(sprintf('J_zz_link%d', idx), J_nom_zz, 'percent', 5);
        
        crossBound = 0.02 * max([J_nom_xx, J_nom_yy, J_nom_zz]);
        J_xy = ureal(sprintf('J_xy_link%d', idx), 0, 'PlusMinus', [-1 1] * crossBound);
        J_yz = ureal(sprintf('J_yz_link%d', idx), 0, 'PlusMinus', [-1 1] * crossBound);
        J_xz = ureal(sprintf('J_xz_link%d', idx), 0, 'PlusMinus', [-1 1] * crossBound);
        
        robot.linkInertia{j} = [J_xx J_xy J_xz; J_xy J_yy J_yz; J_xz J_yz J_zz];
    else
        robot.linkMass{j} = m_nom;
        robot.linkInertia{j} = J_RA_SIM(:,:,j);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%% JOINTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ~exist('q_traj_conf', 'var')
    q_traj_conf = q_traj;
end

if exist('q_i', 'var')
    q_nominale = q_i;
else
    q_nominale = q_traj_conf(:, 1);
end

%q_start = q_traj(:, 1);
q_min_traj = min(q_traj_conf, [], 2);
q_max_traj = max(q_traj_conf, [], 2);
robot.config_SIM = q_nominale.'; % Valore nominale usato per inizializzare Simscape

if UNC_JOINTS
    % Each sample uses its own local joint corridor, defined from the trajectory
    % segment associated with that configuration. This makes the uncertainty
    % depend on the actual arm span along q_traj, instead of using a constant
    % range for all configurations.
    robot.config = { ...
        ureal('Q_1', q_nominale(1), 'Range', [q_min_traj(1)-1e-3 q_max_traj(1)+1e-3], 'AutoSimplify', 'full'), ...
        ureal('Q_2', q_nominale(2), 'Range', [q_min_traj(2)-1e-3 q_max_traj(2)+1e-3], 'AutoSimplify', 'full'), ...
        ureal('Q_3', q_nominale(3), 'Range', [q_min_traj(3)-1e-3 q_max_traj(3)+1e-3], 'AutoSimplify', 'full'), ...
        ureal('Q_4', q_nominale(4), 'Range', [q_min_traj(4)-1e-3 q_max_traj(4)+1e-3], 'AutoSimplify', 'full'), ...
        ureal('Q_5', q_nominale(5), 'Range', [q_min_traj(5)-1e-3 q_max_traj(5)+1e-3], 'AutoSimplify', 'full'), ...
        ureal('Q_6', q_nominale(6), 'Range', [q_min_traj(6)-1e-3 q_max_traj(6)+1e-3], 'AutoSimplify', 'full'), ...
        ureal('Q_7', q_nominale(7), 'Range', [q_min_traj(7)-1e-3 q_max_traj(7)+1e-3], 'AutoSimplify', 'full') ...
    };
else
    robot.config = num2cell(q_nominale);
end

%% Tile
rho = 81.2;        % [kg/m^3] 
Tile_s = 1;        % [m] 
Tile_z = 0.05;     % [m] 
Volume = (3 * sqrt(3) / 2) * (Tile_s^2) * Tile_z;
m_T_SIM = rho * Volume; 

J_T_T_SIM = [m_T_SIM * ( (5/24)*(Tile_s^2) + (1/12)*(Tile_z^2) ), 0, 0; ...
             0, m_T_SIM * ( (5/24)*(Tile_s^2) + (1/12)*(Tile_z^2) ), 0; ...
             0, 0, m_T_SIM * (5/12)*(Tile_s^2)];

if UNC_TILE
    m_T = ureal('m_T', m_T_SIM, 'percent', 2);
    J_T_Txx = ureal('J_T_Txx', J_T_T_SIM(1,1), 'percent', 2);
    J_T_Tyy = ureal('J_T_Tyy', J_T_T_SIM(2,2), 'percent', 2);
    J_T_Tzz = ureal('J_T_Tzz', J_T_T_SIM(3,3), 'percent', 2);
    J_T_Txy = ureal('J_T_Txy', 0, 'PlusMinus', [-1 1]*0.1);
    J_T_Tyz = ureal('J_T_Tyz', 0, 'PlusMinus', [-1 1]*0.1);
    J_T_Txz = ureal('J_T_Txz', 0, 'PlusMinus', [-1 1]*0.1);
    
    J_T_T = [J_T_Txx J_T_Txy J_T_Txz; ...
             J_T_Txy J_T_Tyy J_T_Tyz; ...
             J_T_Txz J_T_Tyz J_T_Tzz];
else
    m_T = m_T_SIM;
    J_T_T = J_T_T_SIM;
end

%% Tiles Stiffness and Damping
K_tile_force = 2000/0.015/5000;
C_tile_force = 2*5*sqrt(K_tile_force/1*2.598*0.2*100);

K_tile_torque = 250/deg2rad(10)/1000;
C_tile_torque = 2*sqrt(K_tile_torque)*0.7;