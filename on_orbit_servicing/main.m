%% data for the on-orbit servicing mission
clear all;
clc;
data;
%% figure 11 in the paper
% figure 11 in the paper can be obtained by running the code "arm_quintictraj_paper.m"
arm_quintictraj_paper;
%% obtaining open loop dynamics

% these arrays are used for gridding the connection uncertainties defining
% if the chaser is connected to the target or not
grabmode_vector=[zeros(1,2550) ones(1,6250) zeros(1,6201)];
grabmode_vector1=[zeros(1,8800) ones(1,6201)];
% these arrays are used for gridding the uncertain spring dampers defining
% if the chaser is connected to the target or not
K1_vector=[zeros(1,2550) 1e10*ones(1,6250) zeros(1,6201)];
D1_vector=[zeros(1,2550) 1e10*ones(1,6250) zeros(1,6201)];
K2_vector=[zeros(1,8800) 1e10*ones(1,6201)];
D2_vector=[zeros(1,8800) 1e10*ones(1,6201)];

% these index values correspond to the 6 different moments of the scenario
% depicted in the on-orbit servicing paper
idx=[1200; 3400; 4450; 5650; 12000; 13050];
idx=idx+1;

% multiplicative uncertainty shown in the paper
delta_mul = eye(3)+0.04*[ultidyn('delta_xx',[1 1]),0.1*ultidyn('delta_xy',[1 1]),0.1*ultidyn('delta_xz',[1 1]);
                      0.1*ultidyn('delta_yx',[1 1]),ultidyn('delta_yy',[1 1]),0.1*ultidyn('delta_yz',[1 1])
                      0.1*ultidyn('delta_zx',[1 1]),0.1*ultidyn('delta_zy',[1 1]),ultidyn('delta_zz',[1 1])];

% additive uncertainty shown in the paper
delta_add=db2mag(-75)*ultidyn('add',[3 3]); 

% open loop dynamics
SDT_USS_paper_SAuncertain;
Plant=ulinearize('SDT_USS_paper_SAuncertain');
% adding additive and mult. uncertainty to the open loop dynamics
PlantU=Plant(4:6,4:6)*delta_mul+delta_add;

% gridding open loop dynamics according to the 6 different instants
% depicted in the paper
Gu_subs_total_SAuncertain_add=usubs(PlantU,  'tan_theta1_div4', tan(traj_q(1, idx)/4),... %call systune here
                   'tan_theta2_div4', tan(traj_q(2, idx)/4),...
                   'tan_theta3_div4', tan(traj_q(3, idx)/4),...
                   'tan_theta4_div4', tan(traj_q(4, idx)/4),...
                   'tan_theta5_div4', tan(traj_q(5, idx)/4),...
                   'tan_theta6_div4', tan(traj_q(6, idx)/4),...
                   'tan_theta7_div4', tan(traj_q_SA(1, idx)/4),...
                   'tan_theta8_div4', tan(-traj_q_SA(1, idx)/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'grabmode', grabmode_vector(1,idx),...
                   'grabmode1',grabmode_vector1(1,idx));
Gu_subs_nom_total_SAuncertain_add=getNominal(Gu_subs_total_SAuncertain_add);

Gu_subs_total_SAuncertain=usubs(Plant,  'tan_theta1_div4', tan(traj_q(1, idx)/4),... %call systune here
                   'tan_theta2_div4', tan(traj_q(2, idx)/4),...
                   'tan_theta3_div4', tan(traj_q(3, idx)/4),...
                   'tan_theta4_div4', tan(traj_q(4, idx)/4),...
                   'tan_theta5_div4', tan(traj_q(5, idx)/4),...
                   'tan_theta6_div4', tan(traj_q(6, idx)/4),...
                   'tan_theta7_div4', tan(traj_q_SA(1, idx)/4),...
                   'tan_theta8_div4', tan(-traj_q_SA(1, idx)/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'grabmode', grabmode_vector(1,idx),...
                   'grabmode1',grabmode_vector1(1,idx));
Gu_subs_nom_total_SAuncertain=getNominal(Gu_subs_total_SAuncertain);

%% playing with the slx file "SIMSCAPE_servicing.slx"
% arm configuration for when the robotic arm docks to the target
chaser.robotic_arm_servicing.init_angles=[0   -3.1416   -0.7854   -2.3562    -1.5708        0];
% run now the slx file to see the configuration
% this file was used for comparison with the SDT file
% this file can also be used for measurements of inertial and mass
% properties of the spacecraft
% this slx file has three blocks related to the target which can be
% commented and uncommented depending on the objective: 
% - Target (chaser is decoupled from the target)
% - Target (first docking has occured)
% - Target (second docking has occured)

% the user can play the geometrical configuration of the robotic arm and
% the different target blocks in the slx file
%% this code generates the figure 9 in the on-orbit servicing journal paper
% the simscape dynamical systems have been precomputed for the sake of
% simplicity
load('simscape')
% these systems can be computed by using the simulink file
% "SIMSCAPE_servicing.slx" and setting up the geometrical configurations
% and the target blocks accordingly
TP_Graph = figure();clf;set(TP_Graph,'defaulttextinterpreter','latex');
hold on; grid on;

subplot(2,3,1)
for i=1:200
    if i==1
        [sv,w]=sigma(usample(Gu_subs_total_SAuncertain(4,4,1,1)),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
        p1=semilogx(w/(2*pi),mag2db(sv),'Color','[0.73 0.83 0.96]','DisplayName','cosine','LineWidth',3);
        hold on;
    else
        [sv,w]=sigma(usample(Gu_subs_total_SAuncertain(4,4,1,1)),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
        semilogx(w/(2*pi),mag2db(sv),'Color','[0.73 0.83 0.96]','HandleVisibility','off')
        hold on;
    end
end
[sv,w]=sigma(Gu_subs_nom_total_SAuncertain(4,4,1,1),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
p2=semilogx(w/(2*pi),mag2db(sv),'-r','LineWidth',1.5,'DisplayName','cosine');
hold on;
[sv,w]=sigma(Gu_simscape1(4,4),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
p3=semilogx(w/(2*pi),mag2db(sv),'--g','LineWidth',2,'DisplayName','cosine');

subplot(2,3,2)
for i=1:200
    [sv,w]=sigma(usample(Gu_subs_total_SAuncertain(4,4,1,2)),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
    semilogx(w/(2*pi),mag2db(sv),'Color','[0.73 0.83 0.96]')
    hold on;
end
[sv,w]=sigma(Gu_subs_nom_total_SAuncertain(4,4,1,2),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
semilogx(w/(2*pi),mag2db(sv),'-r','LineWidth',1.5)
hold on;
[sv,w]=sigma(Gu_simscape2(4,4),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
semilogx(w/(2*pi),mag2db(sv),'--g','LineWidth',2)

subplot(2,3,3)
for i=1:200
    [sv,w]=sigma(usample(Gu_subs_total_SAuncertain(4,4,1,3)),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
    semilogx(w/(2*pi),mag2db(sv),'Color','[0.73 0.83 0.96]')
    hold on;
end
[sv,w]=sigma(Gu_subs_nom_total_SAuncertain(4,4,1,3),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
semilogx(w/(2*pi),mag2db(sv),'-r','LineWidth',1.5)
hold on;
[sv,w]=sigma(Gu_simscape3(4,4),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
semilogx(w/(2*pi),mag2db(sv),'--g','LineWidth',2)

subplot(2,3,4)
for i=1:200
    [sv,w]=sigma(usample(Gu_subs_total_SAuncertain(4,4,1,4)),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
    semilogx(w/(2*pi),mag2db(sv),'Color','[0.73 0.83 0.96]')
    hold on;
end
[sv,w]=sigma(Gu_subs_nom_total_SAuncertain(4,4,1,4),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
semilogx(w/(2*pi),mag2db(sv),'-r','LineWidth',1.5)
hold on;
[sv,w]=sigma(Gu_simscape4(4,4),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
semilogx(w/(2*pi),mag2db(sv),'--g','LineWidth',2)

subplot(2,3,5)
for i=1:200
    [sv,w]=sigma(usample(Gu_subs_total_SAuncertain(4,4,1,5)),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
    semilogx(w/(2*pi),mag2db(sv),'Color','[0.73 0.83 0.96]')
    hold on;
end
[sv,w]=sigma(Gu_subs_nom_total_SAuncertain(4,4,1,5),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
semilogx(w/(2*pi),mag2db(sv),'-r','LineWidth',1.5)
hold on;
[sv,w]=sigma(Gu_simscape5(4,4),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
semilogx(w/(2*pi),mag2db(sv),'--g','LineWidth',2)

subplot(2,3,6)
for i=1:200
    [sv,w]=sigma(usample(Gu_subs_total_SAuncertain(4,4,1,6)),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
    semilogx(w/(2*pi),mag2db(sv),'Color','[0.73 0.83 0.96]')
    hold on;
end
[sv,w]=sigma(Gu_subs_nom_total_SAuncertain(4,4,1,6),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
semilogx(w/(2*pi),mag2db(sv),'-r','LineWidth',1.5)
hold on;
[sv,w]=sigma(Gu_simscape6(4,4),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
semilogx(w/(2*pi),mag2db(sv),'--g','LineWidth',2)

[~, objh] = legend([p1 p2 p3],{'SDT - uncertain ($${\Delta}_{real}$$)' , 'SDT - nominal','Simscape'},'FontSize',30,'Orientation','horizontal','Interpreter','latex');
objhl = findobj(objh, 'type', 'line');
set(objhl, 'LineWidth', 10);
set(gca,'linewidth',1.5)
set(gcf,'color','w');
%% this code generates the figure 15 in the journal paper
% different subsets of uncertainty are sampled and ploted as shown in the
% paper

inertiamass_uncertainty={'target_I_xx','target_I_yy','target_I_zz','target_mass'};
add_uncertainty={'add'};
modal_uncertainty={'wnbig1','wntarget1'};
multiplicative_uncertainty={'delta_xy','delta_xz','delta_yx','delta_yz','delta_zx','delta_zy','delta_xx','delta_yy','delta_zz'};
TP_Graph = figure();clf;set(TP_Graph,'defaulttextinterpreter','latex');
hold on; grid on;

for i=1:200
    if i==1
        [sv,w]=sigma(usample(getNominalExcept(Gu_subs_total_SAuncertain_add(1,1,1,2),modal_uncertainty)),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
        p1=semilogx(w/(2*pi),mag2db(sv),'Color','[0.73 0.83 0.96]','DisplayName','cosine','LineWidth',1.5);
        hold on;
    else
        [sv,w]=sigma(usample(getNominalExcept(Gu_subs_total_SAuncertain_add(1,1,1,2),modal_uncertainty)),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
        semilogx(w/(2*pi),mag2db(sv),'Color','[0.73 0.83 0.96]','HandleVisibility','off')
        hold on;
    end
end

for i=1:200
    if i==1
        [sv,w]=sigma(usample(getNominalExcept(Gu_subs_total_SAuncertain_add(1,1,1,2),add_uncertainty)),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
        p2=semilogx(w/(2*pi),mag2db(sv),'Color','[0.49 0.18 0.56]','DisplayName','cosine','LineWidth',1.5);
        hold on;
    else
        [sv,w]=sigma(usample(getNominalExcept(Gu_subs_total_SAuncertain_add(1,1,1,2),add_uncertainty)),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
        semilogx(w/(2*pi),mag2db(sv),'Color','[0.49 0.18 0.56]','HandleVisibility','off')
        hold on;
    end
end

for i=1:200
    if i==1
        [sv,w]=sigma(usample(getNominalExcept(Gu_subs_total_SAuncertain_add(1,1,1,2),multiplicative_uncertainty)),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
        p3=semilogx(w/(2*pi),mag2db(sv),'Color','[0.00 1.00 0.00]','DisplayName','cosine','LineWidth',1.5);
        hold on;
    else
        [sv,w]=sigma(usample(getNominalExcept(Gu_subs_total_SAuncertain_add(1,1,1,2),multiplicative_uncertainty)),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
        semilogx(w/(2*pi),mag2db(sv),'Color','[0.00 1.00 0.00]','HandleVisibility','off')
        hold on;
    end
end

for i=1:200
    if i==1
        [sv,w]=sigma(usample(getNominalExcept(Gu_subs_total_SAuncertain_add(1,1,1,2),inertiamass_uncertainty)),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
        p4=semilogx(w/(2*pi),mag2db(sv),'Color','[0 0 0]','DisplayName','cosine','LineWidth',1.5);
        hold on;
    else
        [sv,w]=sigma(usample(getNominalExcept(Gu_subs_total_SAuncertain_add(1,1,1,2),inertiamass_uncertainty)),logspace(log10(1e-1),log10(1e2),1e3)*2*pi);
        semilogx(w/(2*pi),mag2db(sv),'Color','[0 0 0]','HandleVisibility','off')
        hold on;
    end
end

[~, objh] = legend([p1 p2 p3 p4],{'Modal $${\Delta}_{mod}$$' , 'Additive $${\Delta}_{add}$$','Multiplicative $${\Delta}_{mul}$$','Mechanical $${\Delta}_{mec}$'},'FontSize',30,'Orientation','vertical','Interpreter','latex');
xlabel('Frequency ($$Hz$$)','Fontsize',24);
ylabel('Gain ($$dB$$)','Fontsize',24);
xscale log
objhl = findobj(objh, 'type', 'line');
set(objhl, 'LineWidth', 8);
%% this code generates the figure 12 in the journal paper

idx_inertia=1:20:size(traj_q,2); 
idx_time=(idx_inertia-1)/10;
uncertainty={'tan_theta1_div4','tan_theta2_div4','tan_theta3_div4','tan_theta4_div4',...
    'tan_theta5_div4','tan_theta6_div4','tan_theta7_div4','tan_theta8_div4','tan_theta9_div4','tan_theta10_div4',...
    'grabmode','grabmode1','target_I_xx','target_I_yy','target_I_zz'};

% compute the nominal open loop system dynamics for several grid points
Gu_subs_total_SAuncertain_inertia=usubs(Plant,  'tan_theta1_div4', tan(traj_q(1, idx_inertia)/4),... %call systune here
                   'tan_theta2_div4', tan(traj_q(2, idx_inertia)/4),...
                   'tan_theta3_div4', tan(traj_q(3, idx_inertia)/4),...
                   'tan_theta4_div4', tan(traj_q(4, idx_inertia)/4),...
                   'tan_theta5_div4', tan(traj_q(5, idx_inertia)/4),...
                   'tan_theta6_div4', tan(traj_q(6, idx_inertia)/4),...
                   'tan_theta7_div4', tan(traj_q_SA(1, idx_inertia)/4),...
                   'tan_theta8_div4', tan(-traj_q_SA(1, idx_inertia)/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'grabmode', grabmode_vector(1,idx_inertia),...
                   'grabmode1',grabmode_vector1(1,idx_inertia));
Gu_subs_nom_total_SAuncertain_inertia=getNominal(Gu_subs_total_SAuncertain_inertia);

% compute the inertia for different grid points from the dc gain
inertia_tensor=dcgain(inv(Gu_subs_nom_total_SAuncertain_inertia(:,:,:,:)));

ixx=squeeze(inertia_tensor(4,4,:,:));
iyy=squeeze(inertia_tensor(5,5,:,:));
izz=squeeze(inertia_tensor(6,6,:,:));
ixy=squeeze(inertia_tensor(4,5,:,:));
izx=squeeze(inertia_tensor(4,6,:,:));
iyz=squeeze(inertia_tensor(5,6,:,:));

% plot of the changing inertias figure        
figure;
subplot(2,1,1)
plot(idx_time,ixx,'r')
hold on
plot(idx_time,iyy,'b')
plot(idx_time,izz,'g')
title('')
subplot(2,1,2)
plot(idx_time,ixy)
hold on
plot(idx_time,izx)
plot(idx_time,iyz)
title('')

%% figure 13 in the journal paper
% figure 13 in the paper can be obtained by running the code "MTG_model_complete_paper.m"

MTG_model_complete_paper;
%% this code generates the figure 10 in the journal paper

% compute OCP5 to make sure the closed loop kinematic chain (second docking phase in the paper) is solved for
% the right geometrical configuration of the system
% OCP5 is defined by the distance vector between the reference point O,
% which in this case is the center of mass of the rigid hub of the chaser
% spacecraft, and the connection point P5, which in this case is 
OCP5_computation;
% the OCP5 computation can also be done by using the slx file
% 'SIMSCAPE_servicing.slx' by using a distance sensor accordingly

% second docking phase in the paper
SDT_USS_paper_springmass_KD8;
angles=[  -0.2618   -1.0996    2.0944   -4.1364    -1.3090     0];
chaser.robotic_arm_servicing.init_angles=angles;
springmass_docking8=ulinearize('SDT_USS_paper_springmass_KD8');
% first docking phase in the paper
SDT_USS_paper_springmass_KD7;
angles=[       0   -3.1416   -0.7854   -2.3562    -1.5708      0];
chaser.robotic_arm_servicing.init_angles=angles;
springmass_docking7=ulinearize('SDT_USS_paper_springmass_KD7');

sysfinal_docking8 = red_slow(springmass_docking8,2e-4);    

% using usubs to get the systems representing the plots in fig. 10 in the
% journal paper
inertia_uncertainty_names_all={'tan_theta7_div4','tan_theta8_div4','tan_theta9_div4','tan_theta10_div4',...
    'tan_theta5_div4','tan_theta6_div4','tan_theta7_div4','tan_theta15_div4','tan_theta16_div4','K1','D1'};
springmass_docking7_nom=getNominalExcept(springmass_docking7, inertia_uncertainty_names_all);
springmass_plot1=usubs(springmass_docking7_nom, 'tan_theta7_div4', tan(0/4),...
                   'tan_theta8_div4', tan(0/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'K1', [0.1,1000:1000:1e5],...
                   'D1', 100,...
                   'K2', 0,...
                   'D2', 0);

springmass_plot2=usubs(sysfinal_docking8, 'tan_theta7_div4', tan(0/4),...
                   'tan_theta8_div4', tan(0/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'K1', 1e5,...
                   'D1', 100,...
                   'K2', [0.1,1000:1000:1e5],...
                   'D2', 100);

springmass_plot3=usubs(sysfinal_docking8, 'tan_theta7_div4', tan(0/4),...
                   'tan_theta8_div4', tan(0/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'K1', [1e5:-1000:0.1,0.1],...
                   'D1', 100,...
                   'K2', 1e5,...
                   'D2', 100);

% plotting figure
figure;
fv=logspace(log10(0.1),log10(100),1000)*2*pi;
col=plasma(120);
col=col(1:101,:);

subplot(1,3,1)
for i=1:101
    [sv,w]=sigma(springmass_plot1(5,5,1,i),fv);
    semilogx(w/(2*pi),mag2db(sv),'Color',col(i,:),'LineWidth',1)
    hold on;  
end
cmap = colormap(col) ; %Create Colormap
cbh = colorbar ; %Create Colorbar
cbh.Label.String = 'Gain (dB)';
cbh.Ticks = linspace(0, 1, 11) ; %Create 8 ticks from zero to 1
cbh.TickLabels = num2cell([0.1,1,2,3,4,5,6,7,8,9,10]) ;    %Replace the labels of these 8 ticks with the numbers 1 to 8

subplot(1,3,2)
for i=1:101
    [sv,w]=sigma(springmass_plot2(5,5,1,i),fv);
    semilogx(w/(2*pi),mag2db(sv),'Color',col(i,:),'LineWidth',1)
    hold on;  
end
cmap = colormap(col) ; %Create Colormap
cbh = colorbar ; %Create Colorbar
cbh.Label.String = 'Gain (dB)';
cbh.Ticks = linspace(0, 1, 11) ; %Create 8 ticks from zero to 1
cbh.TickLabels = num2cell([0.1,1,2,3,4,5,6,7,8,9,10]) ;    %Replace the labels of these 8 ticks with the numbers 1 to 8

subplot(1,3,3)
for i=1:101
    [sv,w]=sigma(springmass_plot3(5,5,1,i),fv);
    semilogx(w/(2*pi),mag2db(sv),'Color',col(101-(i-1),:),'LineWidth',1)
    hold on;  
end
cmap = colormap(col) ; %Create Colormap
cbh = colorbar ; %Create Colorbar
cbh.Label.String = 'Gain (dB)';
cbh.Ticks = linspace(0, 1, 11) ; %Create 8 ticks from zero to 1
cbh.TickLabels = num2cell([0.1,1,2,3,4,5,6,7,8,9,10]) ;    %Replace the labels of these 8 ticks with the numbers 1 to 8

%% CONTROL DESIGN, BOTH BASELINE AND HINF
% the control design is presented in the code 'control_synthesis_paper_final.m'
control_synthesis_paper_final;
%% MU ANALYSIS FOR THE SERVICING MISSION (no spring dampers)
% this mu analysis is done without spring dampers
% the model considers only the uncertain on/off uncertain connections which
% allow us to compute the dynamics of the system when the chaser is
% decoupled from the target but also when they are coupled, as explained in
% the paper
% presented in the code 'muanalysis_paper.m'
muanalysis_paper;
%% MU PLOTS
% the code for the mu plots in the paper are provided in the code 'muplot.m'
% this code uses some .mat files so that it is still possible to plot
% figures even if the code in 'muanalysis_paper.m' was not used
muplot;
%% MU ANALYSIS FOR THE SERVICING MISSION (with spring dampers)
% the same done in the code 'muanalysis_paper.m' should be done but for the
% following dynamic model given by the variable name 'PlantUspring'
PlantUspring=springmass_docking7(4:6,4:6)*delta_mul+delta_add;
% then this open loop model should be used in the slx file 'OL_model_control_synthesis_paper_SAuncertain_add_mul_noweights'
% for stability and in the slx file
% 'OL_model_control_synthesis_paper_SAuncertain_add_mul' for performance mu
% analysis computations
% the follow the instructions in the paper the system should be gridded
% over different values of the uncertainty K1, D1=100 and all the real
% parametric uncertainties are kept uncertain
% then a code very similar to 'muanalysis_paper.m' can be used
%% MU PLOTS WITH SPRING DAMPERS
% code can be found in 'muplot.m'
%% SLOSHING
% the example with sloshing that was presented at the ESA GNC
% conference can be found in the folder 'sloshing_paper_ESA'