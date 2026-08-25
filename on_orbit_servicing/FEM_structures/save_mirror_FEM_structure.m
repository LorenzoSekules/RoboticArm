options = struct();
options.FileName = 'openscad_new_scenario/hexagon_mirror.stl';
options.E = 10e9;      % Young's modulus in Pa, aluminium
options.nu = 0.33;     % Poisson's ratio (nondimensional)
options.rho = 2700/2;    % Mass density in kg/m^3
options.Hmax = 10e-2;
options.Hmin = 2.5e-2;
options.maxModalFreq = 2.5*2*pi;
    
options.faceIDs = [];
%options.faceIDs = [31 36 41 50 57 61 74];
%options.faceIDs = [61 74];


mirror_SIMSCAPE_reduced_FEM = get_reduced_FEM_structure(options);

K_hexa_mirror = mirror_SIMSCAPE_reduced_FEM.K;
wn_hexa_mirror = sqrt(diag(K_hexa_mirror(7:end, 7:end)));
xi_hexa_mirror = 0.01;
C_hexa_mirror = blkdiag(zeros(6), diag(2*xi_hexa_mirror*wn_hexa_mirror));
mirror_SIMSCAPE_reduced_FEM.C = C_hexa_mirror;

m_hexa_mirror= mirror_SIMSCAPE_reduced_FEM.M(1,1);
GP_star_hexa_mirror = mirror_SIMSCAPE_reduced_FEM.M(1:3,4:6)/m_hexa_mirror;
inertias_hexa_mirror = mirror_SIMSCAPE_reduced_FEM.M(4:6,4:6) + m_hexa_mirror*(GP_star_hexa_mirror)^2;
r_PG_hexa_mirror = [GP_star_hexa_mirror(2,3); GP_star_hexa_mirror(3,1); GP_star_hexa_mirror(1,2)];
mirror_SIMSCAPE_reduced_FEM.r_PG = r_PG_hexa_mirror;
mirror_SIMSCAPE_reduced_FEM.xi = xi_hexa_mirror;
mirror_SIMSCAPE_reduced_FEM.m = m_hexa_mirror;
mirror_SIMSCAPE_reduced_FEM.L =  mirror_SIMSCAPE_reduced_FEM.M(7:end, 1:6);
save('mirror_SIMSCAPE_reduced_FEM');
