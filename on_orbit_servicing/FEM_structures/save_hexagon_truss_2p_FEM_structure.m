%beryllium
options.FileName = 'openscad_new_scenario/hexagon_truss.stl';
options.E = 69e9;     % Young's modulus in Pa, aluminium
options.nu = 0.33;  % Poisson's ratio (nondimensional)
%options.nu = 0.18;
options.rho = 2700;    % Mass density in kg/m^3
options.Hmax = 10e-2;
options.Hmin = 2.5e-2;
options.maxModalFreq = 170*2*pi;

options.faceIDs = [15 36];
%68 center 
%options.faceIDs = [31 36 41 50 57 61 74];
%options.faceIDs = [61 74];


hexagon_truss_2p_SIMSCAPE_reduced_FEM = get_reduced_FEM_structure(options);
K_hexa_truss_2p = hexagon_truss_2p_SIMSCAPE_reduced_FEM.K;
wn_hexa_truss_2p = sqrt(diag(K_hexa_truss_2p(13:end, 13:end)));
xi_hexa_truss_2p = 0.01;
C_hexa_truss_2p = blkdiag(zeros(6), diag(2*xi_hexa_truss_2p*wn_hexa_truss_2p));
hexagon_truss_2p_SIMSCAPE_reduced_FEM.C = C_hexa_truss_2p;

m_hexa_truss_2p = hexagon_truss_2p_SIMSCAPE_reduced_FEM.M(1,1);
GP_star_hexa_truss_2p = hexagon_truss_2p_SIMSCAPE_reduced_FEM.M(1:3,4:6)/m_hexa_truss_2p;
inertias_hexa_truss_2p = hexagon_truss_2p_SIMSCAPE_reduced_FEM.M(4:6,4:6) + m_hexa_truss_2p*(GP_star_hexa_truss_2p)^2;
r_PG_hexa_truss_2p = [GP_star_hexa_truss_2p(2,3); GP_star_hexa_truss_2p(3,1); GP_star_hexa_truss_2p(1,2)];
hexagon_truss_2p_SIMSCAPE_reduced_FEM.r_PG = r_PG_hexa_truss_2p;
hexagon_truss_2p_SIMSCAPE_reduced_FEM.xi = xi_hexa_truss_2p;
hexagon_truss_2p_SIMSCAPE_reduced_FEM.m = m_hexa_truss_2p;
hexagon_truss_2p_SIMSCAPE_reduced_FEM.L =  hexagon_truss_2p_SIMSCAPE_reduced_FEM.M(7:end, 1:6);

save('hexagon_truss_2p_SIMSCAPE_reduced_FEM');
