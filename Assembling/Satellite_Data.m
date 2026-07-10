
%% Main Body Data

% define main body dimensions (meters)
main_body_x = 2.2;   % length along x-axis
main_body_y = 3;   % length along y-axis
main_body_z = 5;   % length along z-axis

m_B =       ureal('m_B',4000,'percent',10);     % [kg] Mass central body
m_B_SIM =   usubs(m_B,'m_B',m_B.NominalValue);  % [kg] Mass central body (used in simscape)

J_B_Bxx = ureal('J_B_Bxx',m_B_SIM/12*(main_body_y^2+main_body_z^2),'percent',10); % [kg*m^2] Moment of inertia (xx) of the central body B
J_B_Byy = ureal('J_B_Byy',m_B_SIM/12*(main_body_x^2+main_body_z^2),'percent',10); % [kg*m^2] Moment of inertia (yy) of the central body B
J_B_Bzz = ureal('J_B_Bzz',m_B_SIM/12*(main_body_y^2+main_body_x^2),'percent',10); % [kg*m^2] Moment of inertia (zz) of the central body B
J_B_Bxy =       ureal('J_B_Bxy',0,'PlusMinus',[-1 1]);  %  [kg*m^2] Cross moment of inertia (xy) of the central body B
J_B_Byz =       ureal('J_B_Byz',0,'PlusMinus',[-1 1]); %  [kg*m^2] Cross moment of inertia (yz) of the central body B
J_B_Bxz =       ureal('J_B_Bxz',0,'PlusMinus',[-1 1]);   %  [kg*m^2] Cross moment of inertia (xz) of the central body B

J_B_B = [J_B_Bxx J_B_Bxy J_B_Bxz;...
        J_B_Bxy J_B_Byy J_B_Byz;...
        J_B_Bxz J_B_Byz J_B_Bzz];  % Moment of inertia of the central body B in R_b wrt point B [kg*m^2] 
                        % Note the order in Simscape for product of inertia is given as
                        % J_B_B(2,3) J_B_B(1,3) J_B_B(1,2)

J_B_B_SIM =   usubs(J_B_B,'J_B_Bxx',J_B_Bxx.NominalValue,'J_B_Byy',J_B_Byy.NominalValue,'J_B_Bzz',J_B_Bzz.NominalValue,...
                    'J_B_Bxy',J_B_Bxy.NominalValue,'J_B_Byz',J_B_Byz.NominalValue,'J_B_Bxz',J_B_Bxz.NominalValue);  % [kg*m^2] Moment of inertia of the central

CoM_variation_x = ureal('CoMx',0,"PlusMinus",[-10e-3 10e-3]); % Less than 10-15 mm (x-y) except the launcher axis (around 20 mm)
CoM_variation_y = ureal('CoMy',0,"PlusMinus",[-10e-3 10e-3]);
CoM_variation_z = ureal('CoMz',0,"PlusMinus",[-20e-3 20e-3]);

CoM_Variation_simscape=[CoM_variation_x.NominalValue;CoM_variation_y.NominalValue;CoM_variation_z.NominalValue]; % Variation of CoM used in the Simscape model

%% Container Data

Container_y = sqrt(3)+0.5;
Container_x = 1.5;
Container_z = sqrt(3)+0.5;


m_C =       ureal('m_C',100,'percent',10);     % [kg] Mass central body
m_C_SIM =   usubs(m_C,'m_C',m_C.NominalValue);  % [kg] Mass central body (used in simscape)

J_C_Cxx = ureal('J_C_Cxx',m_C_SIM/12*(Container_y^2+Container_z^2),'percent',10); % [kg*m^2] Moment of inertia (xx) of the central body B
J_C_Cyy = ureal('J_C_Cyy',m_C_SIM/12*(Container_x^2+Container_z^2),'percent',10); % [kg*m^2] Moment of inertia (yy) of the central body B
J_C_Czz = ureal('J_C_Czz',m_C_SIM/12*(Container_y^2+Container_x^2),'percent',10); % [kg*m^2] Moment of inertia (zz) of the central body C
J_C_Cxy =       ureal('J_C_Cxy',0,'PlusMinus',[-1 1]);  %  [kg*m^2] Cross moment of inertia (xy) of the central body B
J_C_Cyz =       ureal('J_C_Cyz',0,'PlusMinus',[-1 1]); %  [kg*m^2] Cross moment of inertia (yz) of the central body B
J_C_Cxz =       ureal('J_C_Cxz',0,'PlusMinus',[-1 1]);   %  [kg*m^2] Cross moment of inertia (xz) of the central Cody B

J_C_C = [J_C_Cxx J_C_Cxy J_C_Cxz;...
        J_C_Cxy J_C_Cyy J_C_Cyz;...
        J_C_Cxz J_C_Cyz J_C_Czz];  % Moment of inertia of the central body B in R_b wrt point B [kg*m^2] 
                        % Note the order in Simscape for product of inertia is given as
                        % J_C_C(2,3) J_C_C(1,3) J_C_C(1,2)

J_C_C_SIM =   usubs(J_C_C,'J_C_Cxx',J_C_Cxx.NominalValue,'J_C_Cyy',J_C_Cyy.NominalValue,'J_C_Czz',J_C_Czz.NominalValue,...
                    'J_C_Cxy',J_C_Cxy.NominalValue,'J_C_Cyz',J_C_Cyz.NominalValue,'J_C_Cxz',J_C_Cxz.NominalValue);  % [kg*m^2] Moment of inertia of the central


%%  Little Bricks Data

Brick_y = 0.2;
Brick_x = 0.2;
Brick_z = 0.4;


m_P =       ureal('m_P',50,'percent',2);     % [kg] Mass central body
m_P_SIM =   usubs(m_P,'m_P',m_P.NominalValue);  % [kg] Mass central body (used in simscape)

J_P_Pxx = ureal('J_P_Pxx',m_P_SIM/12*(Brick_y^2+Brick_z^2),'percent',2); % [kg*m^2] Moment of inertia (xx) of the central body B
J_P_Pyy = ureal('J_P_Pyy',m_P_SIM/12*(Brick_x^2+Brick_z^2),'percent',2); % [kg*m^2] Moment of inertia (yy) of the central body B
J_P_Pzz = ureal('J_P_Pzz',m_P_SIM/12*(Brick_y^2+Brick_x^2),'percent',2); % [kg*m^2] Moment of inertia (zz) of the central body C
J_P_Pxy =       ureal('J_P_Pxy',0,'PlusMinus',[-1 1]*0.1);  %  [kg*m^2] Cross moment of inertia (xy) of the central body B
J_P_Pyz =       ureal('J_P_Pyz',0,'PlusMinus',[-1 1]*0.1); %  [kg*m^2] Cross moment of inertia (yz) of the central body B
J_P_Pxz =       ureal('J_P_Pxz',0,'PlusMinus',[-1 1]*0.1);   %  [kg*m^2] Cross moment of inertia (xz) of the central Cody B

J_P_P = [J_P_Pxx J_P_Pxy J_P_Pxz;...
        J_P_Pxy J_P_Pyy J_P_Pyz;...
        J_P_Pxz J_P_Pyz J_P_Pzz];  % Moment of inertia of the central body B in R_b wrt point B [kg*m^2] 
                        % Note the order in Simscape for product of inertia is given as
                        % J_P_P(2,3) J_P_P(1,3) J_P_P(1,2)

J_P_P_SIM =   usubs(J_P_P,'J_P_Pxx',J_P_Pxx.NominalValue,'J_P_Pyy',J_P_Pyy.NominalValue,'J_P_Pzz',J_P_Pzz.NominalValue,...
                    'J_P_Pxy',J_P_Pxy.NominalValue,'J_P_Pyz',J_P_Pyz.NominalValue,'J_P_Pxz',J_P_Pxz.NominalValue);  % [kg*m^2] Moment of inertia of the central



% %% Flexible Beam
% 
% % length_beam = 4.7;
% % width_beam = 0.2;
% % height_beam = 0.2;
% % 
% % Cross_section_beam = width_beam*height_beam;
% % Iy_beam = (height_beam*width_beam.^3)/12;
% % Iz_beam = (width_beam*height_beam.^3)/12;
% 
% length_beam = 4.7;
% radius_beam = 0.1;
% Cross_section_beam = pi*radius_beam.^2;
% Iy_beam = (pi * radius_beam^4) / 4; % Moment of inertia about the y-axis
% Iz_beam = (pi * radius_beam^4) / 4; % Moment of inertia about the z-axis

%% Flex beam Nastran

% FEM model name
f06_boom='boom';
bdf_boom='boom';

% Interface points
boom.pointP  =       1;          % Attachment node of boom (grid point ID on bdf file)
boom.pointC =   11;         % Not used since we use a 1-port approach for the boom (6488 corresponds to grid point ID on bdf file of the tip node of the boom)
boom.damping_ratio =  0.003;  % Common damping ratio
boom.n_modes =    10;         % Number of modes for each boom
unc_freq_boom =   10;         % common uncertain (percentage) on natural frequency
boom.n_unc = 4;               % Number of modes considered uncertain

% boom.MPCunc = 20;              % Uncertaintiy on the modal participation factor of the solar arrays
% boom.n_MPCunc = 3;              % Number of uncertaintiy on the modal participation factor of the solar arrays

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
% This is used as an equivalent density for each cylindrical segment.
rho_arm = 1650; % [kg/m^3]

% Only the non-zero links are modeled as cylinders: 1, 3, 5, and 7.
activeLinks = find(robot.d ~= 0);
robot.linkLength = robot.d(activeLinks);
robot.linkRadius = robot.radius * ones(1, numel(activeLinks)); % [m] equivalent cylindrical radius

% Two aggregate variables only: one container for the 4 masses and one for the 4 inertia tensors.
robot.linkMass = cell(1, numel(activeLinks));
robot.linkInertia = cell(1, numel(activeLinks));

m_RA_SIM = zeros(1,numel(activeLinks));
J_RA_SIM = zeros(3,3,numel(activeLinks));

for i = 1:numel(activeLinks)
        idx = activeLinks(i); % Actual robot.d index: 1, 3, 5, 7
        L_cyl = robot.linkLength(i);
        r_cyl = robot.linkRadius(i);

        % Nominal mass from equivalent cylindrical volume
        m_name = sprintf('m_link%d', idx);
        robot.linkMass{i} = ureal(m_name, rho_arm * pi * r_cyl^2 * L_cyl, 'percent', 5);

        % Keep the same pattern used for the other components: nominal ureal -> usubs -> inertia.
        m_RA_SIM(i) = usubs(robot.linkMass{i}, m_name, robot.linkMass{i}.NominalValue);

        J_xx = ureal(sprintf('J_xx_link%d', idx), (m_RA_SIM(i) / 12) * (3 * r_cyl^2 + L_cyl^2), 'percent', 5);
        J_yy = ureal(sprintf('J_yy_link%d', idx), (m_RA_SIM(i) / 12) * (3 * r_cyl^2 + L_cyl^2), 'percent', 5);
        J_zz = ureal(sprintf('J_zz_link%d', idx), (m_RA_SIM(i) / 2)  * (r_cyl^2), 'percent', 5);

        % Small cross products of inertia remain uncertain to preserve the same modeling style as the other sections.
        crossBound = 0.02 * max([J_xx.NominalValue, J_yy.NominalValue, J_zz.NominalValue]);
        J_xy = ureal(sprintf('J_xy_link%d', idx), 0, 'PlusMinus', [-1 1] * crossBound);
        J_yz = ureal(sprintf('J_yz_link%d', idx), 0, 'PlusMinus', [-1 1] * crossBound);
        J_xz = ureal(sprintf('J_xz_link%d', idx), 0, 'PlusMinus', [-1 1] * crossBound);

        robot.linkInertia{i} = [J_xx J_xy J_xz;...
                                J_xy J_yy J_yz;...
                                J_xz J_yz J_zz];

        J_RA_SIM(:,:,i) = [J_xx.NominalValue J_xy.NominalValue J_xz.NominalValue;...
                           J_xy.NominalValue J_yy.NominalValue J_yz.NominalValue;...
                           J_xz.NominalValue J_yz.NominalValue J_zz.NominalValue];
end


q_start = q_traj(:, 1);
q_min_traj = min(q_traj, [], 2);
q_max_traj = max(q_traj, [], 2);

% Initial angular configuration of the RA using the tan(theta/4) formalism. Format : [tand(Latitude) tand(Longitude)] 
% Setting the gimbal span the entire trajectory 
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
Tile_s = 1;        % [m] Side length of the regular hexagon (Adjust this to match your footprint)
Tile_z = 0.05;     % [m] Thickness/Height of the panel (Using the ~30.5mm calculated earlier)

% Calculate Volume of a regular hexagonal prism: V = (3*sqrt(3)/2) * s^2 * h
Volume = (3 * sqrt(3) / 2) * (Tile_s^2) * Tile_z;

% Calculate Nominal Mass
m_nominal = rho * Volume; 

% --- UNCERTAIN MASS ---
m_T =       ureal('m_T', m_nominal, 'percent', 2);     % [kg] Mass central body
m_T_SIM =   usubs(m_T,'m_T',m_T.NominalValue);         % [kg] Mass central body (used in simscape)

% --- UNCERTAIN MOMENTS OF INERTIA ---
% For a regular hexagonal prism with flat face in XY plane, extruded along Z
J_T_Txx = ureal('J_T_Txx', m_T_SIM * ( (5/24)*(Tile_s^2) + (1/12)*(Tile_z^2) ), 'percent', 2); % [kg*m^2] Moment of inertia (xx)
J_T_Tyy = ureal('J_T_Tyy', m_T_SIM * ( (5/24)*(Tile_s^2) + (1/12)*(Tile_z^2) ), 'percent', 2); % [kg*m^2] Moment of inertia (yy)
J_T_Tzz = ureal('J_T_Tzz', m_T_SIM * (5/12)*(Tile_s^2), 'percent', 2);                         % [kg*m^2] Moment of inertia (zz)

% --- UNCERTAIN CROSS MOMENTS OF INERTIA ---
% Symmetrical regular hexagons have nominal cross inertias of 0
J_T_Txy =       ureal('J_T_Txy',0,'PlusMinus',[-1 1]*0.1);  % [kg*m^2] Cross moment of inertia (xy)
J_T_Tyz =       ureal('J_T_Tyz',0,'PlusMinus',[-1 1]*0.1);  % [kg*m^2] Cross moment of inertia (yz)
J_T_Txz =       ureal('J_T_Txz',0,'PlusMinus',[-1 1]*0.1);  % [kg*m^2] Cross moment of inertia (xz)

% --- INERTIA TENSOR MATRIX ---
J_T_T = [J_T_Txx J_T_Txy J_T_Txz;...
         J_T_Txy J_T_Tyy J_T_Tyz;...
         J_T_Txz J_T_Tyz J_T_Tzz];  % Moment of inertia of the central body B in R_b wrt point B [kg*m^2] 
                                    % Note the order in Simscape for product of inertia is given as
                                    % J_T_T(2,3) J_T_T(1,3) J_T_T(1,2)

% --- SIMSCAPE INERTIA EVALUATION ---
J_T_T_SIM =   usubs(J_T_T,'J_T_Txx',J_T_Txx.NominalValue,'J_T_Tyy',J_T_Tyy.NominalValue,'J_T_Tzz',J_T_Tzz.NominalValue,...
                    'J_T_Txy',J_T_Txy.NominalValue,'J_T_Tyz',J_T_Tyz.NominalValue,'J_T_Txz',J_T_Txz.NominalValue);

%% Tiles
K_tile_force = 2000/0.015/5000;
C_tile_force = 2*5*sqrt(K_tile_force/2500*2.598*0.2*100);

K_tile_torque = 250/deg2rad(10)/1000;
C_tile_torque = 2*1*sqrt(K_tile_torque/1000*0.541266*100)*0.7/50;

