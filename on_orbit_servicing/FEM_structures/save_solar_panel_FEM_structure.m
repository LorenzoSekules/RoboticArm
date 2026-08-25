options = struct();
options.FileName = '3d_models/simple_solar_panel.stl';
options.E = 70e4;     % Young's modulus in Pa
options.nu = 0.33;     % Poisson's ratio (nondimensional)
options.rho = 2700;    % Mass density in kg/m^3
options.Hmax = 5e-2;
options.Hmin = 2e-2;
options.maxModalFreq = 1*2*pi;
    
    
options.faceIDs = [ ];
%options.faceIDs = [41 2 8 14 20];

solar_panel_reduced_FEM = get_reduced_FEM_structure(options);




K = solar_panel_reduced_FEM.K;
wn = sqrt(diag(K(7:end, 7:end)));
xi = 0.01;
C = blkdiag(zeros(6), diag(2*xi*wn));
solar_panel_reduced_FEM.C = C;


m = solar_panel_reduced_FEM.M(1,1);
GP_star = solar_panel_reduced_FEM.M(1:3,4:6)/m;
inertias = solar_panel_reduced_FEM.M(4:6,4:6) + m*(GP_star)^2;
r_PG = [GP_star(2,3); GP_star(3,1); GP_star(1,2)];
solar_panel_reduced_FEM.r_PG = r_PG;
solar_panel_reduced_FEM.xi = xi;
solar_panel_reduced_FEM.m = m;
solar_panel_reduced_FEM.L =  solar_panel_reduced_FEM.M(7:end, 1:6);

save('solar_panel_reduced_FEM');
