Satellite_Data
load('best_trajectory2.mat')
open('Robot3_SDT');
Gu=ulinearize('Robot3_SDT');
Gum=minreal(Gu);
%% Simulink Simscape
Name_SIM = 'Robot3_SIMSCAPE';
open(Name_SIM)
G_SIMSCAPE_nominal=linearize(Name_SIM);

M0=inv(dcgain(Gu(1:8,1:8)));
M0_SIMSCAPE=inv(dcgain(G_SIMSCAPE_nominal(1:8,1:8)));
IatB=M0(4:6,4:6);

ComparisonDCgain=M0_SIMSCAPE-M0;
disp(ComparisonDCgain)

figure()
sigma(Gum.NominalValue(1:8,1:8)-G_SIMSCAPE_nominal(1:8,1:8))
norm(sigma(Gum.NominalValue(1:8,1:8)-G_SIMSCAPE_nominal(1:8,1:8)),'inf')

%%
figure()
sigma(Gum.NominalValue(1:8,1:8))
hold on
sigma(G_SIMSCAPE_nominal(1:8,1:8))

legend('SDT','Simscape')