
%% Main Body Data

% define main body dimensions (meters)
main_body_x = 2.2;   % length along x-axis
main_body_y = 3;   % length along y-axis
main_body_z = 5;   % length along z-axis

m_B =       ureal('m_B',1000,'percent',10);     % [kg] Mass central body
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


m_C =       ureal('m_C',500,'percent',10);     % [kg] Mass central body
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


%%  Little Brick Data

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



%% Flexible Beam

% length_beam = 4.7;
% width_beam = 0.2;
% height_beam = 0.2;
% 
% Cross_section_beam = width_beam*height_beam;
% Iy_beam = (height_beam*width_beam.^3)/12;
% Iz_beam = (width_beam*height_beam.^3)/12;

length_beam = 4.7;
radius_beam = 0.1;
Cross_section_beam = pi*radius_beam.^2;
Iy_beam = (pi * radius_beam^4) / 4; % Moment of inertia about the y-axis
Iz_beam = (pi * radius_beam^4) / 4; % Moment of inertia about the z-axis

%% Flex beam Nastran

% FEM model name
f06_boom='boom';
bdf_boom='boom';

% Interface points
boom.pointP  =       1;          % Attachment node of boom (grid point ID on bdf file)
boom.pointC =   [];         % Not used since we use a 1-port approach for the boom (6488 corresponds to grid point ID on bdf file of the tip node of the boom)
boom.damping_ratio =  0.003;  % Common damping ratio
boom.n_modes =    10;         % Number of modes for each boom
unc_freq_boom =   10;         % common uncertain (percentage) on natural frequency
boom.n_unc = 6;               % Number of modes considered uncertain

% boom.MPCunc = 20;              % Uncertaintiy on the modal participation factor of the solar arrays
% boom.n_MPCunc = 3;              % Number of uncertaintiy on the modal participation factor of the solar arrays

boom.MPCunc = 0;              % Uncertaintiy on the modal participation factor of the solar arrays
boom.n_MPCunc = 0;              % Number of uncertaintiy on the modal participation factor of the solar arrays

% Extraction of ROM matrix for Simscape Multibody Flex. Reduced order model
[coord_boom,Mrofs_boom,Krofs_boom,Drofs_boom,flagFatal]=Nastran2ROFS(strcat(f06_boom,'.f06'),strcat(bdf_boom,'.bdf'),boom.damping_ratio,boom.pointP,boom.pointC,boom.n_modes); 
