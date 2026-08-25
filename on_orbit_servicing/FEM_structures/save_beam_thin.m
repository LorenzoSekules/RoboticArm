options = struct();
options.FileName = '3d_models/thin_beam.stl';
options.E = 20.85e9;      % Young's modulus in Pa, aluminium
options.nu = 20.85e9/2/8.0192e9-1;     % Poisson's ratio (nondimensional)
options.rho = 4275.3;    % Mass density in kg/m^3
options.Hmax = 5e-2;
options.Hmin = 1e-2;
options.maxModalFreq = 15*2*pi;
    
options.faceIDs = [2];
%options.faceIDs = [31 36 41 50 57 61 74];
%options.faceIDs = [61 74];


beam_truss_4p_SIMSCAPE_reduced_FEM = get_reduced_FEM_structure(options);
K_hexa_beam_truss_4p = beam_truss_4p_SIMSCAPE_reduced_FEM.K;
wn_hexa_beam_truss_4p = sqrt(diag(K_hexa_beam_truss_4p(7:end, 7:end)));
xi_hexa_beam_truss_4p = 0.01;
C_hexa_beam_truss_4p = blkdiag(zeros(6), diag(2*xi_hexa_beam_truss_4p*wn_hexa_beam_truss_4p));
beam_truss_4p_SIMSCAPE_reduced_FEM.C = C_hexa_beam_truss_4p;

m_hexa_beam_truss_4p= beam_truss_4p_SIMSCAPE_reduced_FEM.M(1,1);
GP_star_hexa_beam_truss_4p = beam_truss_4p_SIMSCAPE_reduced_FEM.M(1:3,4:6)/m_hexa_beam_truss_4p;
inertias_hexa_beam_truss_4p = beam_truss_4p_SIMSCAPE_reduced_FEM.M(4:6,4:6) + m_hexa_beam_truss_4p*(GP_star_hexa_beam_truss_4p)^2;
r_PG_hexa_beam_truss_4p = [GP_star_hexa_beam_truss_4p(2,3); GP_star_hexa_beam_truss_4p(3,1); GP_star_hexa_beam_truss_4p(1,2)];
beam_truss_4p_SIMSCAPE_reduced_FEM.r_PG = r_PG_hexa_beam_truss_4p;
beam_truss_4p_SIMSCAPE_reduced_FEM.xi = xi_hexa_beam_truss_4p;
beam_truss_4p_SIMSCAPE_reduced_FEM.m = m_hexa_beam_truss_4p;
beam_truss_4p_SIMSCAPE_reduced_FEM.L =  beam_truss_4p_SIMSCAPE_reduced_FEM.M(7:end, 1:6);
%save('beam_truss_4p_SIMSCAPE_reduced_FEM');
