Satellite_Data
Tile1_Placement = 3;
Tile2_Placement = 3;
Tile3_Placement = 3;
Tile4_Placement = 3;
Tile5_Placement = 3;
Tile6_Placement = 3;
Tile7_Placement = 3;
load('best_trajectory2.mat')
open('Robot3_SDT');
Gu=ulinearize('Robot3_SDT');
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