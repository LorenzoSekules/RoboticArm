options = struct();
options.FileName = 'openscad_new_scenario/hexagon_truss_mirror.stl';
options.E = 69e9;      % Young's modulus in Pa, aluminium
options.nu = 0.33;     % Poisson's ratio (nondimensional)
options.rho = 2700;    % Mass density in kg/m^3
options.Hmax = 10e-2;
options.Hmin = 5e-2;
options.maxModalFreq = 45*2*pi;
    
options.faceIDs = [119 19 614 153];
%options.faceIDs = [31 36 41 50 57 61 74];
%options.faceIDs = [61 74];


mirror_truss_4p_SIMSCAPE_reduced_FEM = get_reduced_FEM_structure(options);

K_hexa_mirror_truss_4p = mirror_truss_4p_SIMSCAPE_reduced_FEM.K;
wn_hexa_mirror_truss_4p = sqrt(diag(K_hexa_mirror_truss_4p(25:end, 25:end)));
xi_hexa_mirror_truss_4p = 0.01;
C_hexa_mirror_truss_4p = blkdiag(zeros(24), diag(2*xi_hexa_mirror_truss_4p*wn_hexa_mirror_truss_4p));
mirror_truss_4p_SIMSCAPE_reduced_FEM.C = C_hexa_mirror_truss_4p;

m_hexa_mirror_truss_4p= mirror_truss_4p_SIMSCAPE_reduced_FEM.M(1,1);
GP_star_hexa_mirror_truss_4p = mirror_truss_4p_SIMSCAPE_reduced_FEM.M(1:3,4:6)/m_hexa_mirror_truss_4p;
inertias_hexa_mirror_truss_4p = mirror_truss_4p_SIMSCAPE_reduced_FEM.M(4:6,4:6) + m_hexa_mirror_truss_4p*(GP_star_hexa_mirror_truss_4p)^2;
r_PG_hexa_mirror_truss_4p = [GP_star_hexa_mirror_truss_4p(2,3); GP_star_hexa_mirror_truss_4p(3,1); GP_star_hexa_mirror_truss_4p(1,2)];
mirror_truss_4p_SIMSCAPE_reduced_FEM.r_PG = r_PG_hexa_mirror_truss_4p;
mirror_truss_4p_SIMSCAPE_reduced_FEM.xi = xi_hexa_mirror_truss_4p;
mirror_truss_4p_SIMSCAPE_reduced_FEM.m = m_hexa_mirror_truss_4p;
mirror_truss_4p_SIMSCAPE_reduced_FEM.L =  mirror_truss_4p_SIMSCAPE_reduced_FEM.M(7:end, 1:6);
save('mirror_truss_4p_SIMSCAPE_reduced_FEM');
