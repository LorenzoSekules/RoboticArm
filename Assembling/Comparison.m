load('best_trajectory2.mat')
Satellite_Data

Tile1_Placement = 1;
Tile2_Placement = 1;
Tile3_Placement = 1;
Tile4_Placement = 1;
Tile5_Placement = 1;
Tile6_Placement = 1;
Tile7_Placement = 1;

open('Robot4_SDT');
Gu=ulinearize('Robot4_SDT');
Gum=minreal(Gu);

%% Simulink Simscape
Name_SIM = 'Robot3_SIMSCAPE';
open(Name_SIM)
G_SIMSCAPE_nominal=linearize(Name_SIM);

M0=inv(dcgain(Gu(1:13,1:13)));
M0_SIMSCAPE=inv(dcgain(G_SIMSCAPE_nominal(1:13,1:13)));
IatB=M0(4:6,4:6);

ComparisonDCgain=M0_SIMSCAPE-M0;
disp(ComparisonDCgain)

figure()
sigma(Gum.NominalValue(1:13,1:13)-G_SIMSCAPE_nominal(1:13,1:13))
norm(sigma(Gum.NominalValue(1:13,1:13)-G_SIMSCAPE_nominal(1:13,1:13)),'inf')

%%
figure()
sigma(Gum.NominalValue(1:13,1:13))
hold on
sigma(G_SIMSCAPE_nominal(1:13,1:13))

legend('SDT','Simscape')