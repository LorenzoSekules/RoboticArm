%silicon
options = struct();
options.FileName = 'openscad_new_scenario/solar_panel_target.stl';
options.E = 10e9;     % Young's modulus in Pa, aluminium
options.nu = 0.33;     % Poisson's ratio (nondimensional)
options.rho = 2700/2;    % Mass density in kg/m^3
options.Hmax = 3e-2;
options.Hmin = 1e-2;
options.maxModalFreq = 20*2*pi;
    
options.faceIDs = [53];
%options.faceIDs = [41 2 8 14 20];

solar_panel_target_reduced_FEM = get_reduced_FEM_structure(options);


K_target = solar_panel_target_reduced_FEM.K;
wn_target = sqrt(diag(K_target(7:end, 7:end)));
xi_target = 0.01;
C_target = blkdiag(zeros(6), diag(2*xi_target*wn_target));
solar_panel_target_reduced_FEM.C = C_target;


m_target = solar_panel_target_reduced_FEM.M(1,1);
GP_star_target = solar_panel_target_reduced_FEM.M(1:3,4:6)/m_target;
inertias_target = solar_panel_target_reduced_FEM.M(4:6,4:6) + m_target*(GP_star_target)^2;
r_PG_target = [GP_star_target(2,3); GP_star_target(3,1); GP_star_target(1,2)];
solar_panel_target_reduced_FEM.r_PG = r_PG_target;
solar_panel_target_reduced_FEM.xi = xi_target;
solar_panel_target_reduced_FEM.m = m_target;
solar_panel_target_reduced_FEM.L =  solar_panel_target_reduced_FEM.M(7:end, 1:6);

%save('solar_panel_target_reduced_FEM');