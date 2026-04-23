Satellite_Data
load('best_trajectory2.mat')
open('trial_SDT');
Gu=ulinearize('trial_SDT');
Gum=minreal(Gu);

%% Simulink Simscape
Name_SIM = 'trial_SIM';
open(Name_SIM)
G_SIMSCAPE_nominal=linearize(Name_SIM);

M0=inv(dcgain(Gu(1:6,1:6)));
M0_SIMSCAPE=inv(dcgain(G_SIMSCAPE_nominal(1:6,1:6)));
IatB=M0(4:6,4:6);

ComparisonDCgain=M0_SIMSCAPE-M0;
disp(ComparisonDCgain)

figure()
sigma(Gum.NominalValue(1:6,1:12)-G_SIMSCAPE_nominal(1:6,1:12))
norm(sigma(Gum.NominalValue(1:6,1:12)-G_SIMSCAPE_nominal(1:6,1:12)),'inf')

%%
figure()
sigma(Gum.NominalValue(1:6,1:12))
hold on
sigma(G_SIMSCAPE_nominal(1:6,1:12))

legend('SDT','Simscape')