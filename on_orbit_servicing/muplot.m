%% figure 16 in the journal paper
% .mat files present in the folder 'mudata_OOS'
figure;
subplot(1,3,1)
load('complete.mat')
[X,Y] = meshgrid(fv/(2*pi),time_traj);
surf(X,Y,1./SV,'EdgeColor','none');
view([0 90]);
axis tight
xlabel('Frequency (Hz)');
ylabel('Time (sec)');
set(gca,'XScale','log');
c = colorbar;
c.Label.String = 'Gain (dB)';
colormap('plasma');

subplot(1,3,2)
load('add.mat')
plot(time_traj,1./stabmarg_LowerBound,'-r','LineWidth',2)
hold on
load('inertia.mat')
plot(time_traj,1./stabmarg_LowerBound,'-b','LineWidth',2)
hold on
load('mul.mat')
plot(time_traj,1./stabmarg_LowerBound,'Color','[0.93 0.69 0.13]','LineWidth',2)
hold on
load('nominal.mat')
plot(time_traj,1./stabmarg_LowerBound,'Color','[0.49 0.18 0.56]','LineWidth',2)
hold on
load('wn.mat')
plot(time_traj,1./stabmarg_LowerBound,'Color','[0 0.5 0]','LineWidth',2)
hold on
load('complete.mat')
plot(time_traj,1./stabmarg_LowerBound,'Color','[0 0 0]','LineWidth',2)

subplot(1,3,3)
load('add.mat')
worstBounds = zeros(length(fv),1);
for i=1:size(SV,2)
    worstBounds(i)=max(1./SV(:,i));
end
semilogx(fv/(2*pi),worstBounds,'-r','LineWidth',2)
hold on
load('inertia.mat')
worstBounds = zeros(length(fv),1);
for i=1:size(SV,2)
    worstBounds(i)=max(1./SV(:,i));
end
semilogx(fv/(2*pi),worstBounds,'-b','LineWidth',2)
hold on
load('mul.mat')
worstBounds = zeros(length(fv),1);
for i=1:size(SV,2)
    worstBounds(i)=max(1./SV(:,i));
end
semilogx(fv/(2*pi),worstBounds,'Color','[0.93 0.69 0.13]','LineWidth',2)
hold on
load('nominal.mat')
worstBounds = zeros(length(fv),1);
for i=1:size(SV,2)
    worstBounds(i)=max(1./SV(:,i));
end
semilogx(fv/(2*pi),worstBounds,'Color','[0.49 0.18 0.56]','LineWidth',2)
hold on
load('wn.mat')
worstBounds = zeros(length(fv),1);
for i=1:size(SV,2)
    worstBounds(i)=max(1./SV(:,i));
end
semilogx(fv/(2*pi),worstBounds,'Color','[0 0.5 0]','LineWidth',2)
hold on
load('complete.mat')
worstBounds = zeros(length(fv),1);
for i=1:size(SV,2)
    worstBounds(i)=max(1./SV(:,i));
end
semilogx(fv/(2*pi),worstBounds,'Color','[0 0 0]','LineWidth',2)
%% figure 18 in the journal paper

figure;
subplot(1,2,1)
load('springmass_mu.mat')
plot(time_traj,1./stabmarg_LowerBound,'Color','[0.2362    0.0154    0.6082]','LineWidth',2)

subplot(1,2,2)
load('springmass_mu.mat')
worstBounds = zeros(length(fv),1);
for i=1:size(SV,2)
    worstBounds(i)=max(1./SV(:,i));
end
semilogx(fv/(2*pi),worstBounds,'Color','[0.2362    0.0154    0.6082]','LineWidth',2)
%% figure 17 in the journal paper

TP_Graph = figure();clf;set(TP_Graph,'defaulttextinterpreter','latex');
hold on; grid on;

subplot(1,2,1)
load('add.mat')
worstBounds_wcgain_total13 = zeros(length(fv_wcgain_total13),1);
worstBounds_wcgain_total213 = zeros(length(fv_wcgain_total13),1);
for i=1:size(SV_wcg_total13,2)
    worstBounds_wcgain_total13(i)=max(SV_wcg_total13(:,i));
    worstBounds_wcgain_total213(i)=max(SV2_wcg_total13(:,i));
end
semilogx(fv_wcgain_total13/(2*pi),worstBounds_wcgain_total213,'-r','LineWidth',2)
hold on
load('inertia.mat')
worstBounds_wcgain_total13 = zeros(length(fv_wcgain_total13),1);
worstBounds_wcgain_total213 = zeros(length(fv_wcgain_total13),1);
for i=1:size(SV_wcg_total13,2)
    worstBounds_wcgain_total13(i)=max(SV_wcg_total13(:,i));
    worstBounds_wcgain_total213(i)=max(SV2_wcg_total13(:,i));
end
semilogx(fv_wcgain_total13/(2*pi),worstBounds_wcgain_total213,'-b','LineWidth',2)
hold on
load('mul.mat')
worstBounds_wcgain_total13 = zeros(length(fv_wcgain_total13),1);
worstBounds_wcgain_total213 = zeros(length(fv_wcgain_total13),1);
for i=1:size(SV_wcg_total13,2)
    worstBounds_wcgain_total13(i)=max(SV_wcg_total13(:,i));
    worstBounds_wcgain_total213(i)=max(SV2_wcg_total13(:,i));
end
semilogx(fv_wcgain_total13/(2*pi),worstBounds_wcgain_total213,'Color','[0.93 0.69 0.13]','LineWidth',2)
hold on
load('nominal.mat')
worstBounds_wcgain_total13 = zeros(length(fv_wcgain_total13),1);
worstBounds_wcgain_total213 = zeros(length(fv_wcgain_total13),1);
for i=1:size(SV_wcg_total13,2)
    worstBounds_wcgain_total13(i)=max(SV_wcg_total13(:,i));
    worstBounds_wcgain_total213(i)=max(SV2_wcg_total13(:,i));
end
semilogx(fv_wcgain_total13/(2*pi),worstBounds_wcgain_total213,'Color','[0.49 0.18 0.56]','LineWidth',2)
hold on
load('wn.mat')
worstBounds_wcgain_total13 = zeros(length(fv_wcgain_total13),1);
worstBounds_wcgain_total213 = zeros(length(fv_wcgain_total13),1);
for i=1:size(SV_wcg_total13,2)
    worstBounds_wcgain_total13(i)=max(SV_wcg_total13(:,i));
    worstBounds_wcgain_total213(i)=max(SV2_wcg_total13(:,i));
end
semilogx(fv_wcgain_total13/(2*pi),worstBounds_wcgain_total213,'Color','[0 0.5 0]','LineWidth',2)
hold on
load('complete.mat')
worstBounds_wcgain_total13 = zeros(length(fv_wcgain_total13),1);
worstBounds_wcgain_total213 = zeros(length(fv_wcgain_total13),1);
for i=1:size(SV_wcg_total13,2)
    worstBounds_wcgain_total13(i)=max(SV_wcg_total13(:,i));
    worstBounds_wcgain_total213(i)=max(SV2_wcg_total13(:,i));
end
semilogx(fv_wcgain_total13/(2*pi),worstBounds_wcgain_total213,'Color','[0 0 0]','LineWidth',2)
hold on

subplot(1,2,2)
load("add.mat")
worstBounds_wcgain_total46 = zeros(length(fv_wcgain_total46),1);
worstBounds_wcgain_total246 = zeros(length(fv_wcgain_total46),1);
for i=1:size(SV_wcg_total46,2)
    worstBounds_wcgain_total46(i)=max(SV_wcg_total46(:,i));
    worstBounds_wcgain_total246(i)=max(SV2_wcg_total46(:,i));
end
p1=semilogx(fv_wcgain_total46/(2*pi),worstBounds_wcgain_total246,'-r','LineWidth',2);
hold on
load("inertia.mat")
worstBounds_wcgain_total46 = zeros(length(fv_wcgain_total46),1);
worstBounds_wcgain_total246 = zeros(length(fv_wcgain_total46),1);
for i=1:size(SV_wcg_total46,2)
    worstBounds_wcgain_total46(i)=max(SV_wcg_total46(:,i));
    worstBounds_wcgain_total246(i)=max(SV2_wcg_total46(:,i));
end
p2=semilogx(fv_wcgain_total46/(2*pi),worstBounds_wcgain_total246,'-b','LineWidth',2);
hold on
load("mul.mat")
worstBounds_wcgain_total46 = zeros(length(fv_wcgain_total46),1);
worstBounds_wcgain_total246 = zeros(length(fv_wcgain_total46),1);
for i=1:size(SV_wcg_total46,2)
    worstBounds_wcgain_total46(i)=max(SV_wcg_total46(:,i));
    worstBounds_wcgain_total246(i)=max(SV2_wcg_total46(:,i));
end
p3=semilogx(fv_wcgain_total46/(2*pi),worstBounds_wcgain_total246,'Color','[0.93 0.69 0.13]','LineWidth',2);
hold on
load("nominal.mat")
worstBounds_wcgain_total46 = zeros(length(fv_wcgain_total46),1);
worstBounds_wcgain_total246 = zeros(length(fv_wcgain_total46),1);
for i=1:size(SV_wcg_total46,2)
    worstBounds_wcgain_total46(i)=max(SV_wcg_total46(:,i));
    worstBounds_wcgain_total246(i)=max(SV2_wcg_total46(:,i));
end
p4=semilogx(fv_wcgain_total46/(2*pi),worstBounds_wcgain_total246,'Color','[0.49 0.18 0.56]','LineWidth',2);
hold on
load("wn.mat")
worstBounds_wcgain_total46 = zeros(length(fv_wcgain_total46),1);
worstBounds_wcgain_total246 = zeros(length(fv_wcgain_total46),1);
for i=1:size(SV_wcg_total46,2)
    worstBounds_wcgain_total46(i)=max(SV_wcg_total46(:,i));
    worstBounds_wcgain_total246(i)=max(SV2_wcg_total46(:,i));
end
p5=semilogx(fv_wcgain_total46/(2*pi),worstBounds_wcgain_total246,'Color','[0.00 0.50 0.00]','LineWidth',2);
hold on
load("complete.mat")
worstBounds_wcgain_total46 = zeros(length(fv_wcgain_total46),1);
worstBounds_wcgain_total246 = zeros(length(fv_wcgain_total46),1);
for i=1:size(SV_wcg_total46,2)
    worstBounds_wcgain_total46(i)=max(SV_wcg_total46(:,i));
    worstBounds_wcgain_total246(i)=max(SV2_wcg_total46(:,i));
end
p6=semilogx(fv_wcgain_total46/(2*pi),worstBounds_wcgain_total246,'Color','[0 0 0]','LineWidth',2);
hold on

[~, objh] = legend([p1 p2 p3 p4 p5 p6],{'Additive $$\Delta_{add}$$' , 'Mechanical $$\Delta_{mec}$$','Multiplicative $$\Delta_{mul}$$','Nominal','Modal $$\Delta_{mod}$$','Total $$\Delta_{tot}$$'},'FontSize',30,'Orientation','horizontal','Interpreter','latex');
xlabel('Frequency ($$Hz$$)','Fontsize',24);
ylabel('Gain ($$dB$$)','Fontsize',24);
objhl = findobj(objh, 'type', 'line');
set(objhl, 'LineWidth', 10);
set(gca,'linewidth',1.5)
set(gcf,'color','w');



