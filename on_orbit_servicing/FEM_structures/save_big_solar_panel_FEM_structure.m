%silicon
options = struct();
options.FileName = '3d_models/solar_panel_big.stl';
options.E = 10e9;     % Young's modulus in Pa, aluminium
options.nu = 0.33;     % Poisson's ratio (nondimensional)
options.rho = 2700/2;    % Mass density in kg/m^3
% options.E = 69e9;     % Young's modulus in Pa, aluminium
% options.nu = 0.33;     % Poisson's ratio (nondimensional)
% options.rho = 2700;    % Mass density in kg/m^3
options.Hmax = 10e-2;
options.Hmin = 2.5e-2;
options.maxModalFreq = 50*2*pi;
    
options.faceIDs = [];
%options.faceIDs = [41 2 8 14 20];


solar_panel_big_reduced_FEM = get_reduced_FEM_structure(options);

return
K_big = solar_panel_big_reduced_FEM.K;
wn_big = sqrt(diag(K_big(7:end, 7:end)));
xi_big = 0.01;
C_big = blkdiag(zeros(6), diag(2*xi_big*wn_big));
solar_panel_big_reduced_FEM.C = C_big;


m_big = solar_panel_big_reduced_FEM.M(1,1);
GP_star_big = solar_panel_big_reduced_FEM.M(1:3,4:6)/m_big;
inertias_big = solar_panel_big_reduced_FEM.M(4:6,4:6) + m_big*(GP_star_big)^2;
r_PG_big = [GP_star_big(2,3); GP_star_big(3,1); GP_star_big(1,2)];
solar_panel_big_reduced_FEM.r_PG = r_PG_big;
solar_panel_big_reduced_FEM.xi = xi_big;
solar_panel_big_reduced_FEM.m = m_big;
solar_panel_big_reduced_FEM.L =  solar_panel_big_reduced_FEM.M(7:end, 1:6);

%save('solar_panel_big_reduced_FEM');