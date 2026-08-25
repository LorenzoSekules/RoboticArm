%beryllium
options.FileName = '3d_models/hexagon.stl';
options.E = 10e7;     % Young's modulus in Pa, aluminium
options.nu = 0.33;  % Poisson's ratio (nondimensional)
%options.nu = 0.18;
options.rho = 2700;    % Mass density in kg/m^3
options.Hmax = 10e-2;
options.Hmin = 2.5e-2;
options.maxModalFreq = 1000*2*pi;

options.faceIDs = [37];
%68 center 
%options.faceIDs = [31 36 41 50 57 61 74];
%options.faceIDs = [61 74];



hexagon_SIMSCAPE_reduced_FEM = get_reduced_FEM_structure(options);
K_hexa = hexagon_SIMSCAPE_reduced_FEM.K;
wn_hexa = sqrt(diag(K_hexa(7:end, 7:end)));
xi_hexa = 0.01;
C_hexa = blkdiag(zeros(6), diag(2*xi_hexa*wn_hexa));
hexagon_SIMSCAPE_reduced_FEM.C = C_hexa;

m_hexa = hexagon_SIMSCAPE_reduced_FEM.M(1,1);
GP_star_hexa = hexagon_SIMSCAPE_reduced_FEM.M(1:3,4:6)/m_hexa;
inertias_hexa = hexagon_SIMSCAPE_reduced_FEM.M(4:6,4:6) + m_hexa*(GP_star_hexa)^2;
r_PG_hexa = [GP_star_hexa(2,3); GP_star_hexa(3,1); GP_star_hexa(1,2)];
hexagon_SIMSCAPE_reduced_FEM.r_PG = r_PG_hexa;
hexagon_SIMSCAPE_reduced_FEM.xi = xi_hexa;
hexagon_SIMSCAPE_reduced_FEM.m = m_hexa;
hexagon_SIMSCAPE_reduced_FEM.L =  hexagon_SIMSCAPE_reduced_FEM.M(7:end, 1:6);
save('hexagon_SIMSCAPE_reduced_FEM');
