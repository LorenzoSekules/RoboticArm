options = struct();
options.FileName = '3d_models/solar_panel_medium.stl';
options.E = 10e9;     % Young's modulus in Pa, aluminium
options.nu = 0.33;     % Poisson's ratio (nondimensional)
options.rho = 2700/3;    % Mass density in kg/m^3
options.Hmax = 10e-2;
options.Hmin = 2.5e-2;
options.maxModalFreq = 50*2*pi;
    
options.faceIDs = [173];
%options.faceIDs = [41 2 8 14 20];

solar_panel_medium_reduced_FEM = get_reduced_FEM_structure(options);


K_medium = solar_panel_medium_reduced_FEM.K;
wn_medium = sqrt(diag(K_medium(7:end, 7:end)));
xi_medium = 0.01;
C_medium = blkdiag(zeros(6), diag(2*xi_medium*wn_medium));
solar_panel_medium_reduced_FEM.C = C_medium;


m_medium = solar_panel_medium_reduced_FEM.M(1,1);
GP_star_medium = solar_panel_medium_reduced_FEM.M(1:3,4:6)/m_medium;
inertias_medium = solar_panel_medium_reduced_FEM.M(4:6,4:6) + m_medium*(GP_star_medium)^2;
r_PG_medium = [GP_star_medium(2,3); GP_star_medium(3,1); GP_star_medium(1,2)];
solar_panel_medium_reduced_FEM.r_PG = r_PG_medium;
solar_panel_medium_reduced_FEM.xi = xi_medium;
solar_panel_medium_reduced_FEM.m = m_medium;
solar_panel_medium_reduced_FEM.L =  solar_panel_medium_reduced_FEM.M(7:end, 1:6);

save('solar_panel_medium_reduced_FEM');