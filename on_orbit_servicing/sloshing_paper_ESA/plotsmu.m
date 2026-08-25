%% figure 10 in ESA GNC paper

figure;

subplot(1,3,1)
load('MUTUDO.mat')
[X,Y] = meshgrid(fv/(2*pi),time_traj);
surf(X,Y,1./SV,'EdgeColor','none');
view([0 90]);
axis tight
xlabel('Frequency (Hz)');
ylabel('Time (s)');
zlabel('mu upper bounds');
set(gca,'XScale','log');
c = colorbar;
c.Label.String = 'Gain (dB)';
colormap('plasma');
xlim([10^(-2) 10^1])

subplot(1,3,2)
load('MUTUDO.mat')
plot(time_traj,1./stabmarg_LowerBound,'Color','[0.49 0.18 0.56]','LineWidth',2)
hold on
load('MUSLOSHTARGET.mat')
plot(time_traj,1./stabmarg_LowerBound,'Color','[0 0.5 0]','LineWidth',2)
hold on
load('MUSLOSHCHASER.mat')
plot(time_traj,1./stabmarg_LowerBound,'Color','[0 0 0]','LineWidth',2)
hold on
load('MUFLEXMODES.mat')
plot(time_traj,1./stabmarg_LowerBound,'Color','[0 0 0]','LineWidth',2)

xlabel('Time (s)');
ylabel('Maximum mu bounds');

subplot(1,3,3)
load('MUTUDO.mat')
worstBounds = zeros(length(fv),1);
for i=1:size(SV,2)
    worstBounds(i)=max(1./SV(:,i));
end
semilogx(fv/(2*pi),worstBounds,'Color','[0.49 0.18 0.56]','LineWidth',2)
hold on
load('MUSLOSHTARGET.mat')
worstBounds = zeros(length(fv),1);
for i=1:size(SV,2)
    worstBounds(i)=max(1./SV(:,i));
end
semilogx(fv/(2*pi),worstBounds,'Color','[0 0.5 0]','LineWidth',2)
hold on
load('MUSLOSHCHASER.mat')
worstBounds = zeros(length(fv),1);
for i=1:size(SV,2)
    worstBounds(i)=max(1./SV(:,i));
end
semilogx(fv/(2*pi),worstBounds,'Color','[0 0 0]','LineWidth',2)
hold on
load('MUFLEXMODES.mat')
worstBounds = zeros(length(fv),1);
for i=1:size(SV,2)
    worstBounds(i)=max(1./SV(:,i));
end
semilogx(fv/(2*pi),worstBounds,'Color','[0 0 0]','LineWidth',2)

xlim([10^(-2) 10^1])
xlabel('Frequency (Hz)');
legend('Mechanical unc','Modal unc','Total unc')
%% figure 11 in the ESA GNC paper
% in this case results are normalized, not given in units like in the paper
% just need to multiply by requirements to obtain same effect

TP_Graph = figure();clf;set(TP_Graph,'defaulttextinterpreter','latex');
hold on; grid on;

subplot(1,2,1)
load('MUTUDO.mat')
worstBounds_wcgain_total13 = zeros(length(fv_wcgain_total13),1);
worstBounds_wcgain_total213 = zeros(length(fv_wcgain_total13),1);
for i=1:size(SV_wcg_total13,2)
    worstBounds_wcgain_total13(i)=max(SV_wcg_total13(:,i));
    worstBounds_wcgain_total213(i)=max(SV2_wcg_total13(:,i));
end
semilogx(fv_wcgain_total13/(2*pi),worstBounds_wcgain_total213,'-b','LineWidth',2)
hold on
load('MUSLOSHTARGET.mat')
worstBounds_wcgain_total13 = zeros(length(fv_wcgain_total13),1);
worstBounds_wcgain_total213 = zeros(length(fv_wcgain_total13),1);
for i=1:size(SV_wcg_total13,2)
    worstBounds_wcgain_total13(i)=max(SV_wcg_total13(:,i));
    worstBounds_wcgain_total213(i)=max(SV2_wcg_total13(:,i));
end
semilogx(fv_wcgain_total13/(2*pi),worstBounds_wcgain_total213,'Color','[0 0.5 0]','LineWidth',2)
hold on
load('MUSLOSHCHASER.mat')
worstBounds_wcgain_total13 = zeros(length(fv_wcgain_total13),1);
worstBounds_wcgain_total213 = zeros(length(fv_wcgain_total13),1);
for i=1:size(SV_wcg_total13,2)
    worstBounds_wcgain_total13(i)=max(SV_wcg_total13(:,i));
    worstBounds_wcgain_total213(i)=max(SV2_wcg_total13(:,i));
end
semilogx(fv_wcgain_total13/(2*pi),worstBounds_wcgain_total213,'Color','[0 0 0]','LineWidth',2)
hold on
load('MUFLEXMODES.mat')
worstBounds_wcgain_total13 = zeros(length(fv_wcgain_total13),1);
worstBounds_wcgain_total213 = zeros(length(fv_wcgain_total13),1);
for i=1:size(SV_wcg_total13,2)
    worstBounds_wcgain_total13(i)=max(SV_wcg_total13(:,i));
    worstBounds_wcgain_total213(i)=max(SV2_wcg_total13(:,i));
end
semilogx(fv_wcgain_total13/(2*pi),worstBounds_wcgain_total213,'Color','[0 0 0]','LineWidth',2)

xlim([10^(-2) 10^1])
xlabel('Frequency (Hz)');
ylabel('Max mu bounds on the APE');
title('WITH ROLLOFF')

subplot(1,2,2)
load("MUTUDO.mat")
worstBounds_wcgain_total46 = zeros(length(fv_wcgain_total46),1);
worstBounds_wcgain_total246 = zeros(length(fv_wcgain_total46),1);
for i=1:size(SV_wcg_total46,2)
    worstBounds_wcgain_total46(i)=max(SV_wcg_total46(:,i));
    worstBounds_wcgain_total246(i)=max(SV2_wcg_total46(:,i));
end
p2=semilogx(fv_wcgain_total46/(2*pi),worstBounds_wcgain_total246,'-b','LineWidth',2);
hold on
load("MUSLOSHTARGET.mat")
worstBounds_wcgain_total46 = zeros(length(fv_wcgain_total46),1);
worstBounds_wcgain_total246 = zeros(length(fv_wcgain_total46),1);
for i=1:size(SV_wcg_total46,2)
    worstBounds_wcgain_total46(i)=max(SV_wcg_total46(:,i));
    worstBounds_wcgain_total246(i)=max(SV2_wcg_total46(:,i));
end
p3=semilogx(fv_wcgain_total46/(2*pi),worstBounds_wcgain_total246,'Color','[0.00 0.50 0.00]','LineWidth',2);
hold on
load("MUSLOSHCHASER.mat")
worstBounds_wcgain_total46 = zeros(length(fv_wcgain_total46),1);
worstBounds_wcgain_total246 = zeros(length(fv_wcgain_total46),1);
for i=1:size(SV_wcg_total46,2)
    worstBounds_wcgain_total46(i)=max(SV_wcg_total46(:,i));
    worstBounds_wcgain_total246(i)=max(SV2_wcg_total46(:,i));
end
p4=semilogx(fv_wcgain_total46/(2*pi),worstBounds_wcgain_total246,'Color','[0 0 0]','LineWidth',2);
hold on
load("MUFLEXMODES.mat")
worstBounds_wcgain_total46 = zeros(length(fv_wcgain_total46),1);
worstBounds_wcgain_total246 = zeros(length(fv_wcgain_total46),1);
for i=1:size(SV_wcg_total46,2)
    worstBounds_wcgain_total46(i)=max(SV_wcg_total46(:,i));
    worstBounds_wcgain_total246(i)=max(SV2_wcg_total46(:,i));
end
p5=semilogx(fv_wcgain_total46/(2*pi),worstBounds_wcgain_total246,'Color','[0 0 0]','LineWidth',2);

xlim([10^(-2) 10^1])
xlabel('Frequency (Hz)');
ylabel('Max mu bounds on the control effort');
legend('Mechanical unc','Modal unc','Total unc')

